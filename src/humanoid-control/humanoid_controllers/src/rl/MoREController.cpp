// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/MoREController.h"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <angles/angles.h>
#include "kuavo_common/common/common.h"
#include <ros/package.h>
#include <cmath>
#include <sstream>
#include <thread>

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace ocs2::humanoid;

  namespace
  {
    size_t ovShapeElemCount(const ov::Shape& shape)
    {
      if (shape.empty())
      {
        return 0;
      }
      size_t count = 1;
      for (const auto dim : shape)
      {
        count *= dim;
      }
      return count;
    }

    std::string eigenToShortString(const Eigen::VectorXd& v, int precision = 3)
    {
      std::ostringstream oss;
      oss.setf(std::ios::fixed);
      oss.precision(precision);
      for (int i = 0; i < v.size(); ++i)
      {
        if (i > 0)
        {
          oss << ", ";
        }
        oss << v[i];
      }
      return oss.str();
    }

    /// motion_style_weights one-hot（与 mjlab / ONNX gate 一致）:
    /// 风格1 index=0 → [1,0,0] pose
    /// 风格2 index=1 → [0,1,0] walk + policy arm
    /// 风格3 index=2 → [0,0,1] walk + external arm
    const char* motionStyleHumanName(int idx)
    {
      switch (idx)
      {
        case 0:
          return "style1_pose";
        case 1:
          return "style2_walk_policy_arm";
        case 2:
          return "style3_walk_external_arm";
        default:
          return "unknown";
      }
    }

    std::string motionStyleOneHotString(int idx, int num_gait)
    {
      std::ostringstream oss;
      oss << "[";
      for (int i = 0; i < num_gait; ++i)
      {
        if (i > 0)
        {
          oss << ", ";
        }
        oss << (i == idx ? 1 : 0);
      }
      oss << "]";
      return oss.str();
    }
  }  // namespace

  MoREController::MoREController(const std::string& name,
                                 const std::string& config_file,
                                 ros::NodeHandle& nh,
                                 TopicLogger* ros_logger)
    : RLControllerBase(name, RLControllerType::MORE_CONTROLLER, config_file, nh, ros_logger)
  {
    // 构造函数里 RLControllerBase 已经调用 initializeServices() 和 initializeRLVariables()
  }

  bool MoREController::initialize()
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
    // 动作播放期间不走 stance 保护：设 timeout=0 让 robot_action_active 立即过期，仅影响 MoRE 实例
    double saved_timeout = 0.5;
    ros::param::param<double>("/rl_gait_receiver/robot_action_active_timeout", saved_timeout, 0.5);
    ros::param::set("/rl_gait_receiver/robot_action_active_timeout", 0.001);
    gait_receiver_ = std::make_unique<RlGaitReceiver>(nh_, &initial_cmd_);
    ros::param::set("/rl_gait_receiver/robot_action_active_timeout", saved_timeout);
    // Manager 初始化控制器时只会置 PAUSED，不一定调用派生 pause()；
    // 因此在第一次 resume 前显式禁用，避免非活跃 MoRE 接受复位请求。
    gait_receiver_->setEnabled(false);

    // MoRE 自己持有 VR 躯干退出语义；RlGaitReceiver 只提供通用目标跟踪能力。
    posture_reset_control_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/rl_controller/more_posture_reset", 10,
        &MoREController::postureResetControlCallback, this);

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

    if (is_real_ && use_amp_ruiwo_kpkd_)
    {
      srv_change_motor_param_ =
          nh_.serviceClient<kuavo_msgs::ExecuteArmAction>("/hardware/change_ruiwo_motor_param");
      ROS_INFO("[%s] Ruiwo motor param client ready", name_.c_str());
    }

    // 初始化ankleSolver（从ROS参数获取，如果不存在则使用默认值）
    std::string ankle_solver_type = "5gen";
    if (!nh_.getParam("/ankle_solver_type", ankle_solver_type))
    {
      ROS_WARN("[%s] ankle_solver_type not found in ROS params, using default: %s", name_.c_str(), ankle_solver_type.c_str());
    }
    else
    {
      ROS_INFO("[%s] AnkleSolver type loaded from ROS params: %s", name_.c_str(), ankle_solver_type.c_str());
    }
    ankleSolver_.getconfig(ankle_solver_type);
    ROS_INFO("[%s] AnkleSolver initialized with type: %s", name_.c_str(), ankle_solver_type.c_str());

    // 初始化手臂控制（可选功能）
    std::string urdf_path;
    if (nh_.getParam("/urdfFile", urdf_path))
    {
      ROS_INFO("[%s] Using URDF path from ROS param /urdfFile: %s", name_.c_str(), urdf_path.c_str());
    }
    else
    {
      int robot_version_int = 54;
      nh_.param("/robot_version", robot_version_int, 54);
      const int major = robot_version_int / 10;
      const int minor = robot_version_int % 10;
      const std::string package_path = ros::package::getPath("kuavo_assets");
      const std::string version_str = "biped_s" + std::to_string(major) + std::to_string(minor);
      urdf_path = package_path + "/models/" + version_str + "/urdf/" + version_str + ".urdf";
      ROS_INFO("[%s] Constructed URDF path from robot_version: %s", name_.c_str(), urdf_path.c_str());
    }

    initArmControl(urdf_path);
    initWaistControl();

    // MoRE 模式切换服务：允许外部设置 0/1/2 三种模式
    change_more_mode_srv_ = nh_.advertiseService("/humanoid_controller/change_more_mode",
                                                 &MoREController::changeMoreModeCallback, this);
    // 离线动作播放服务：仅在 pose 风格下执行手臂动作
    execute_arm_action_srv_ = nh_.advertiseService("/humanoid_controller/more_execute_arm_action",
                                                    &MoREController::executeArmActionCallback, this);
    // 系统级手臂动作服务客户端（委托给 humanoid_plan_arm_trajectory 执行）
    system_arm_action_client_ =
        nh_.serviceClient<kuavo_msgs::ExecuteArmAction>("/execute_arm_action");

    if (gait_style_mode_ == "command")
    {
      style_command_sub_ = nh_.subscribe(style_command_topic_, 10, &MoREController::styleCommandCallback, this);
      ROS_INFO("[%s] motion_style_weights from '%s' (Int32 index 0/1/2 → 风格1/2/3 → [1,0,0]/[0,1,0]/[0,0,1])",
               name_.c_str(), style_command_topic_.c_str());
    }
    else
    {
      ROS_INFO("[%s] motion_style_weights mode='%s' (not using command topic)", name_.c_str(),
               gait_style_mode_.c_str());
    }

    quest_joystick_sub_ =
        nh_.subscribe("/quest_joystick_data", 10, &MoREController::questJoystickCallback, this);
    robot_action_state_sub_ =
        nh_.subscribe("/robot_action_state", 10, &MoREController::robotActionStateCallback, this);
    ROS_INFO("[%s] VR control: press right grip + B -> switch to MoRE; hold left X + press right A -> arm/waist mode toggle",
             name_.c_str());

    initialized_ = true;
    ROS_INFO("[%s] MoREController initialized (num_gait=%d, body_dim=%d, history_len=%d, layout=%s)",
             name_.c_str(), num_gait_, policy_body_dim_, obs_history_len_,
             usesThreeGaitExperts() ? "3-style" : "2-style(pose+walk_policy_arm)");
    return true;
  }

  bool MoREController::changeMoreModeCallback(kuavo_msgs::changeArmCtrlMode::Request& req,
                                              kuavo_msgs::changeArmCtrlMode::Response& res)
  {
    const int requested_mode = req.control_mode;

    if (requested_mode < 0 || requested_mode > 2)
    {
      ROS_WARN("[%s] Received invalid MoRE mode %d, valid range is [0, 2]. Keep current mode %d.",
               name_.c_str(), requested_mode, more_mode_);
      res.result = false;
      res.mode = more_mode_;
      res.message = "Invalid MoRE mode, must be 0, 1 or 2";
      return true;
    }

    more_mode_ = requested_mode;

    if (arm_controller_)
    {
      syncArmControllerMode(getCurrentGaitStyleIndex());
    }
    else if (waist_controller_)
    {
      const int idx = getCurrentGaitStyleIndex();
      syncWaistControllerMode(idx, idx != last_synced_gait_style_index_);
      last_synced_gait_style_index_ = idx;
    }

    ROS_INFO("[%s] MoRE arm mode stored=%d, ArmController active=%d, WaistController active=%d (pose style -> mode 2 for VR).",
             name_.c_str(), more_mode_,
             arm_controller_ ? arm_controller_->getMode() : -1,
             waist_controller_ ? waist_controller_->getMode() : -1);

    res.result = true;
    res.mode = more_mode_;
    res.message = "MoRE mode updated successfully";
    return true;
  }

  bool MoREController::executeArmActionCallback(kuavo_msgs::ExecuteArmAction::Request& req,
                                                 kuavo_msgs::ExecuteArmAction::Response& res)
  {
    if (!arm_controller_ || !arm_command_replacement_enabled_)
    {
      res.success = false;
      res.message = "Arm controller not available or disabled";
      return true;
    }

    const int current_style = getCurrentGaitStyleIndex();
    const int current_arm_mode = arm_controller_->getMode();

    // 记录状态 + 启动计时器
    pre_action_style_ = current_style;
    pre_action_arm_mode_ = current_arm_mode;
    last_robot_action_state_ = -1;  // 重置旧状态，防止 checkAndRestoreAfterAction 误触发立即恢复
    robot_action_state_stable_count_ = 0;
    restore_time_ = ros::Time(0);
    action_pending_restore_ = true;
    action_start_time_ = ros::Time::now();

    // 设 walking 标志 + 锁 VR（gap 期间 VR 干扰，等 action_active 首次触发时解锁）
    ros::param::set("/allow_walking_during_arm_action", true);
    if (gait_receiver_) gait_receiver_->setAllowWalkingDuringAction(true);
    if (arm_controller_) arm_controller_->lockExternalTarget(true);

    // if (current_style == 0 && current_arm_mode == 1)  // deprecated: mode1 废弃，style 0 arm 恒为 2
    // {
    //   ROS_INFO("[%s] execute_arm_action '%s': style0/mode1 deprecated",
    //            name_.c_str(), req.action_name.c_str());
    // }
    if (current_style == 0 || current_style == 2)
    {
      // style 0 mode 2 或 style 2: 已在 mode 2，直接播，不恢复
      if (current_arm_mode != 2)
      {
        arm_controller_->changeMode(2);
      }
      ROS_INFO("[%s] execute_arm_action '%s': style%d/mode%d, no restore",
               name_.c_str(), req.action_name.c_str(), current_style, current_arm_mode);
    }
    else if (current_style == 1)
    {
      // style 1 (摆臂走): 切 style 2 → 手臂平滑过渡到动作 → 播 → 恢复 style 1
      style_command_index_ = 2;
      syncArmControllerMode(2);
      ROS_INFO("[%s] execute_arm_action '%s': style1 → style2 (keep walking) → will restore style1",
               name_.c_str(), req.action_name.c_str());
    }

    res.success = true;
    res.message = "Ready for arm action. Auto-triggering /execute_arm_action for '" + req.action_name + "'";

    // 异步触发 Python 端 /execute_arm_action，消除手动两步调用间隙
    // detach 线程不阻塞控制循环，与两步调用设计的"非阻塞"初衷一致
    std::thread([this, action_name = req.action_name]() {
      if (!system_arm_action_client_.waitForExistence(ros::Duration(3.0)))
      {
        ROS_WARN("[%s] /execute_arm_action service not available for auto-trigger of '%s'",
                 name_.c_str(), action_name.c_str());
        return;
      }
      kuavo_msgs::ExecuteArmAction srv;
      srv.request.action_name = action_name;
      if (system_arm_action_client_.call(srv))
      {
        ROS_INFO("[%s] Auto-trigger /execute_arm_action '%s': success=%d",
                 name_.c_str(), action_name.c_str(), srv.response.success);
      }
      else
      {
        ROS_ERROR("[%s] Auto-trigger /execute_arm_action '%s' service call failed",
                  name_.c_str(), action_name.c_str());
      }
    }).detach();

    return true;
  }

  void MoREController::robotActionStateCallback(
      const humanoid_plan_arm_trajectory::RobotActionState::ConstPtr& msg)
  {
    last_robot_action_state_ = msg->state;
  }

  void MoREController::checkAndRestoreAfterAction()
  {
    if (!action_pending_restore_)
      return;

    // 双触发：动作结束信号 或 超时兜底
    const double elapsed = (ros::Time::now() - action_start_time_).toSec();
    const bool action_just_ended = (last_robot_action_state_ >= 0 && last_robot_action_state_ != 1);
    if (!action_just_ended && elapsed < kActionTimeoutSec_)
      return;

    // style0/style2 不需要恢复风格，但需要恢复 arm mode
    const bool needs_style_restore = (pre_action_style_ == 1);
    if (!needs_style_restore)
    {
      // 第一步：动作刚结束，立即恢复手臂模式至动作前状态
      if (restore_time_.isZero())
      {
        const int saved_arm_mode = pre_action_arm_mode_;
        if (arm_controller_)
          arm_controller_->lockExternalTarget(false);
        // style0: 恢复 pose_arm_control_mode_，sync 在 updateInternalState 后用传感器值切 mode
        if (pre_action_style_ == 0 && (saved_arm_mode == 1 || saved_arm_mode == 2))
          pose_arm_control_mode_ = saved_arm_mode;
        restore_time_ = ros::Time::now();  // 释放 resolveMotionStyleIndex / getCurrentGaitStyleIndex 风格锁
        skip_sync_after_restore_ = 0;
        ROS_INFO("[%s] execute_arm_action: arm mode restored (style%d, mode=%d)",
                 name_.c_str(), pre_action_style_, saved_arm_mode);
        return;
      }

      // 第二步：等 Python reset 完成后清理 walking 标志
      if (last_robot_action_state_ == 2)
      {
        robot_action_state_stable_count_++;
        if (robot_action_state_stable_count_ < 5)
          return;
      }
      else if (elapsed < 2.0)
      {
        robot_action_state_stable_count_ = 0;
        return;
      }

      const int saved_pre_action_style = pre_action_style_;
      ros::param::set("/allow_walking_during_arm_action", false);
      if (gait_receiver_)
        gait_receiver_->setAllowWalkingDuringAction(false);
      action_pending_restore_ = false;
      pre_action_style_ = -1;
      pre_action_arm_mode_ = -1;
      restore_time_ = ros::Time(0);
      robot_action_state_stable_count_ = 0;
      ROS_INFO("[%s] execute_arm_action cleanup (style%d): walking flags cleared",
               name_.c_str(), saved_pre_action_style);
      return;
    }

    // style1 需要恢复：先恢复风格/手臂模式，walking 标志延迟清
    if (!restore_time_.isValid() || restore_time_.isZero())
    {
      style_command_index_ = 1;
      syncArmControllerMode(1);
      ROS_INFO("[%s] execute_arm_action done: restored style1", name_.c_str());
      restore_time_ = ros::Time::now();
      skip_sync_after_restore_ = 5;  // 冷却几帧防止 auto 模式立刻又被 VR 数据切回 mode 2
      robot_action_state_stable_count_ = 0;
      return;
    }

    // 等 Python reset 真正结束：state==2 且持续 10 帧 或 超时 3 秒兜底
    if ((ros::Time::now() - restore_time_).toSec() > 3.0)
    {
      ROS_WARN("[%s] action cleanup timeout, force clear walking flags", name_.c_str());
    }
    else if (last_robot_action_state_ == 2)
    {
      robot_action_state_stable_count_++;
      if (robot_action_state_stable_count_ < 10) return;
    }
    else
    {
      robot_action_state_stable_count_ = 0;
      return;
    }

    ros::param::set("/allow_walking_during_arm_action", false);
    if (gait_receiver_)
      gait_receiver_->setAllowWalkingDuringAction(false);
    if (arm_controller_)
      arm_controller_->lockExternalTarget(false);
    action_pending_restore_ = false;
    pre_action_style_ = -1;
    pre_action_arm_mode_ = -1;
    restore_time_ = ros::Time(0);
    ROS_INFO("[%s] execute_arm_action cleanup: walking flags cleared", name_.c_str());
  }

  bool MoREController::loadConfig(const std::string& rlParamFile)
  {
    bool verbose = false;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(rlParamFile, pt);

    auto loadEigenMatrix = [&](const std::string& key, auto& matrix) {
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
    loadEigenMatrix("velocityLimits", velocityLimits_);
    loadEigenMatrix("jointCmdFilterCutoffFreq", jointCmdFilterCutoffFreq_);

    jointCmdFilter_.setParams(dt_, jointCmdFilterCutoffFreq_);
    jointCmdFilterState_.resize(jointCmdFilterCutoffFreq_.size());
    jointCmdFilterState_.setZero();
    loadEigenMatrix("jointCmdFilterState", jointCmdFilterState_);

    initialStateRL_.resize(12 + defalutJointPosRL_.size());
    initialStateRL_ << defaultBaseStateRL_, defalutJointPosRL_;

    loadData::loadCppDataType(rlParamFile, "actionScale", actionScale_);
    loadData::loadCppDataType(rlParamFile, "clipActions", clipActions_);
    loadData::loadCppDataType(rlParamFile, "withArm", withArm_);
    {
      loadData::loadCppDataType(rlParamFile, "inferenceFrequency", inference_frequency_);
      if (inference_frequency_ > 1e-6)
      {
        policy_control_step_dt_ = 1.0 / inference_frequency_;
      }
      try
      {
        loadData::loadCppDataType(rlParamFile, "policyControlStepDt", policy_control_step_dt_);
      }
      catch (const std::exception&)
      {
      }
      const double expected_inference_hz = 1.0 / policy_control_step_dt_;
      if (std::abs(inference_frequency_ - expected_inference_hz) > 0.5)
      {
        ROS_WARN("[%s] inferenceFrequency=%.1f Hz != 1/policyControlStepDt=%.1f Hz; "
                 "obs_history cadence should match mjlab env.step (timestep*decimation)",
                 name_.c_str(), inference_frequency_, expected_inference_hz);
      }
      ROS_INFO("[%s] MoRE policy timing: policyControlStepDt=%.4f s, inferenceFrequency=%.1f Hz "
               "(runtime obs/history sampled at this cadence)",
               name_.c_str(), policy_control_step_dt_, inference_frequency_);
    }
    loadData::loadCppDataType(rlParamFile, "defaultBaseHeightControl", defaultBaseHeightControl_);
    try
    {
      loadData::loadCppDataType(rlParamFile, "defaultBaseXOffsetControl", defaultBaseXOffsetControl_);
    }
    catch (const std::exception& e)
    {
      ROS_WARN("[%s] defaultBaseXOffsetControl not found, using default: %f", name_.c_str(),
               defaultBaseXOffsetControl_);
    }

    loadData::loadCppDataType(rlParamFile, "numGait", num_gait_);
    loadData::loadCppDataType(rlParamFile, "obsHistoryLen", obs_history_len_);
    loadData::loadPtreeValue(pt, gait_style_mode_, "gaitStyleMode", false);
    loadData::loadPtreeValue(pt, default_gait_style_index_, "defaultGaitStyleIndex", false);
    loadData::loadPtreeValue(pt, style_command_topic_, "moreStyleCommandTopic", false);
    style_command_index_ = default_gait_style_index_;
    loadData::loadPtreeValue(pt, posture_nonzero_gait_style_index_, "postureNonzeroGaitStyleIndex", false);
    if (pt.find("postureCommandLimits") != pt.not_found())
    {
      loadData::loadPtreeValue(pt, posture_height_min_, "postureCommandLimits.heightMin", false);
      loadData::loadPtreeValue(pt, posture_height_max_, "postureCommandLimits.heightMax", false);
      loadData::loadPtreeValue(pt, posture_bend_min_, "postureCommandLimits.bendMin", false);
      loadData::loadPtreeValue(pt, posture_bend_max_, "postureCommandLimits.bendMax", false);
    }
    ROS_INFO("[%s] posture_commands from /cmd_pose (RlGaitReceiver), gait_gate_style_idx=%d (-1=off), "
             "height=[%.3f, %.3f] m, bend=[%.3f, %.3f] rad",
             name_.c_str(), posture_nonzero_gait_style_index_, posture_height_min_, posture_height_max_,
             posture_bend_min_, posture_bend_max_);
    loadData::loadPtreeValue(pt, onnx_actions_only_, "onnxActionsOnly", true);
    loadData::loadPtreeValue(pt, log_policy_obs_, "logPolicyObs", false);
    loadData::loadPtreeValue(pt, log_policy_obs_throttle_sec_, "logPolicyObsThrottleSec", false);

    loadData::loadPtreeValue(pt, use_jointcmd_filter_, "use_jointcmd_filter", true);
    loadData::loadPtreeValue(pt, use_amp_ruiwo_kpkd_, "use_amp_ruiwo_kpkd", false);

    bool arm_command_replacement_enabled = false;
    loadData::loadPtreeValue(pt, arm_command_replacement_enabled, "use_external_arm_controller", false);
    use_external_arm_controller(arm_command_replacement_enabled);
    ROS_INFO("[%s] Arm command replacement enabled: %s", name_.c_str(),
             arm_command_replacement_enabled ? "true" : "false");

    if (arm_command_replacement_enabled && jointArmNum_ > 0)
    {
      loadData::loadPtreeValue(pt, arm_max_tracking_velocity_, "armVelocityLimit.maxTrackingVelocity", false);
      loadData::loadPtreeValue(pt, arm_tracking_error_threshold_, "armVelocityLimit.trackingErrorThreshold", false);
      loadData::loadPtreeValue(pt, arm_mode_interpolation_velocity_, "armVelocityLimit.modeInterpolationVelocity", false);
      ROS_INFO("[%s] Arm control parameters loaded: max_velocity=%.3f rad/s, error_threshold=%.3f rad, "
               "mode_interpolation_velocity=%.3f rad/s",
               name_.c_str(), arm_max_tracking_velocity_, arm_tracking_error_threshold_,
               arm_mode_interpolation_velocity_);

      loadData::loadPtreeValue(pt, arm_rl_takeover_blend_enabled_, "armRlTakeoverBlend.enabled", false);
      loadData::loadPtreeValue(pt, arm_rl_takeover_blend_duration_, "armRlTakeoverBlend.duration", 0.3);
      loadData::loadPtreeValue(pt, arm_zero_action_in_standing_, "armRlTakeoverBlend.zeroActionInStanding", false);
      arm_takeover_blender_.configure(arm_rl_takeover_blend_enabled_, arm_rl_takeover_blend_duration_);
      ROS_INFO("[%s] Arm RL takeover blend: enabled=%s, duration=%.3f s, zero_action_in_standing=%s",
               name_.c_str(),
               arm_rl_takeover_blend_enabled_ ? "true" : "false",
               arm_rl_takeover_blend_duration_,
               arm_zero_action_in_standing_ ? "true" : "false");
    }

    loadData::loadPtreeValue(pt, posture_smooth_enabled_, "postureSmooth.enabled", true);
    loadData::loadPtreeValue(pt, posture_smooth_duration_, "postureSmooth.duration", 0.2);
    ROS_INFO("[%s] Posture smooth: enabled=%s, duration=%.3f s",
             name_.c_str(),
             posture_smooth_enabled_ ? "true" : "false",
             posture_smooth_duration_);

    bool waist_command_replacement_enabled = false;
    loadData::loadPtreeValue(pt, waist_command_replacement_enabled, "use_external_waist_controller", false);
    use_external_waist_controller(waist_command_replacement_enabled);
    ROS_INFO("[%s] Waist command replacement enabled: %s", name_.c_str(),
             waist_command_replacement_enabled ? "true" : "false");

    if (waist_command_replacement_enabled && waistNum_ > 0)
    {
      loadData::loadPtreeValue(pt, waist_mode_interpolation_velocity_, "waistControllerParam.modeInterpolationVelocity",
                               false);
      loadData::loadPtreeValue(pt, waist_mode2_cutoff_freq_, "waistControllerParam.mode2CutoffFreq", false);
      double waist_kp_default = 10.0;
      double waist_kd_default = 2.0;
      loadData::loadPtreeValue(pt, waist_kp_default, "waistControllerParam.kp", false);
      loadData::loadPtreeValue(pt, waist_kd_default, "waistControllerParam.kd", false);
      waist_kp_from_config_ = Eigen::VectorXd::Constant(waistNum_, waist_kp_default);
      waist_kd_from_config_ = Eigen::VectorXd::Constant(waistNum_, waist_kd_default);
      ROS_INFO("[%s] Waist control parameters loaded: mode_interpolation_velocity=%.3f rad/s, "
               "mode2_cutoff_freq=%.1f Hz, kp=%.1f, kd=%.1f",
               name_.c_str(), waist_mode_interpolation_velocity_, waist_mode2_cutoff_freq_, waist_kp_default,
               waist_kd_default);
    }

    loadData::loadPtreeValue(pt, waist_zero_tracking_enabled_, "waistZeroTrackingEnabled", false);
    ROS_INFO("[%s] Waist zero tracking in walking enabled: %s", name_.c_str(),
             waist_zero_tracking_enabled_ ? "true" : "false");

    if (pt.find("yawCompensation") != pt.not_found())
    {
      loadData::loadPtreeValue(pt, yaw_compensation_enabled_, "yawCompensation.enabled", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_bias_, "yawCompensation.xBias", false);
      loadData::loadPtreeValue(pt, yaw_compensation_threshold_, "yawCompensation.threshold", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_velocity_threshold_, "yawCompensation.xVelocityThreshold", false);
      loadData::loadPtreeValue(pt, yaw_compensation_separate_enabled_, "yawCompensation.enableSeparateCompensation",
                               false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_bias_clockwise_, "yawCompensation.xBiasClockwise", false);
      loadData::loadPtreeValue(pt, yaw_compensation_x_bias_counterclockwise_, "yawCompensation.xBiasCounterclockwise",
                               false);
      ROS_INFO("[%s] YAW compensation loaded: enabled=%s, xBias=%.4f, threshold=%.4f, xVelThreshold=%.4f, "
               "separate=%s",
               name_.c_str(), yaw_compensation_enabled_ ? "true" : "false", yaw_compensation_x_bias_,
               yaw_compensation_threshold_, yaw_compensation_x_velocity_threshold_,
               yaw_compensation_separate_enabled_ ? "true" : "false");
      if (yaw_compensation_enabled_ && yaw_compensation_separate_enabled_)
      {
        ROS_INFO("[%s] YAW separate compensation: clockwise=%.4f, counterclockwise=%.4f", name_.c_str(),
                 yaw_compensation_x_bias_clockwise_, yaw_compensation_x_bias_counterclockwise_);
      }
    }
    else
    {
      ROS_INFO("[%s] YAW compensation not found in config, using defaults (disabled)", name_.c_str());
    }

    std::string networkModelFile;
    loadData::loadCppDataType(rlParamFile, "networkModelFile", networkModelFile);
    nh_.getParam("/network_model_file", networkModelPath_);
    networkModelPath_ = networkModelPath_ + networkModelFile;

    numSingleObs_ = 0;
    singleInputDataKeys_.clear();
    singleInputDataID_.clear();
    for (const auto& pair : pt)
    {
      if (pair.first != "singleInputData")
      {
        continue;
      }
      for (const auto& pair2 : pair.second)
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

    if (numSingleObs_ <= 0)
    {
      ROS_ERROR("[%s] numSingleObs_ invalid", name_.c_str());
      return false;
    }

    policy_body_dim_ = numSingleObs_;
    policy_obs_dim_ = num_gait_ + policy_body_dim_;

    singleInputData_.resize(policy_body_dim_);
    singleInputData_.setZero();
    policy_obs_.resize(policy_obs_dim_);
    policy_obs_.setZero();
    onnx_obs_buffer_.resize(policy_obs_dim_);
    onnx_obs_buffer_.setZero();
    history_tensor_.resize(obs_history_len_ * policy_body_dim_);
    history_tensor_.setZero();
    initHistoryBuffer();

    compiled_model_ = core_.compile_model(networkModelPath_, "CPU");
    const auto inputs = compiled_model_.inputs();
    if (pt.find("hasObsHistoryEncoder") != pt.not_found())
    {
      loadData::loadPtreeValue(pt, has_obs_history_encoder_, "hasObsHistoryEncoder", false);
      if (has_obs_history_encoder_ && inputs.size() < 2)
      {
        ROS_WARN("[%s] hasObsHistoryEncoder=true but ONNX has only %zu input(s), forcing false",
                 name_.c_str(), inputs.size());
        has_obs_history_encoder_ = false;
      }
      ROS_INFO("[%s] hasObsHistoryEncoder from config=%s (ONNX inputs=%zu), obs history encoder %s",
               name_.c_str(), has_obs_history_encoder_ ? "true" : "false", inputs.size(),
               has_obs_history_encoder_ ? "enabled" : "disabled");
    }
    else
    {
      has_obs_history_encoder_ = (inputs.size() >= 2);
      ROS_INFO("[%s] hasObsHistoryEncoder not in config, auto-detected from %zu input(s): %s",
               name_.c_str(), inputs.size(), has_obs_history_encoder_ ? "enabled" : "disabled");
    }

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
    for (const auto& item : cmdInitalList)
    {
      loadData::loadPtreeValue(pt, initial_cmd_.*(item.second), prefixCommandData_ + ".inital." + item.first, false);
    }
    for (const auto& item : cmdScaleList)
    {
      loadData::loadPtreeValue(pt, initial_cmd_.*(item.second), prefixCommandData_ + ".scale." + item.first, false);
    }

    loadData::loadPtreeValue(pt, cmdVelLineXNegScale_, "commandData.scale.cmdVelLineXNegScale", false);

    ROS_INFO("[%s] velocity limits: vx=[%.3f, %.3f] m/s, wz=±%.3f rad/s (neg_x=pos_x*%.3f)",
             name_.c_str(), -velocityLimits_(0) * cmdVelLineXNegScale_, velocityLimits_(0),
             velocityLimits_(3), cmdVelLineXNegScale_);
    ROS_INFO("[%s] loadConfig done. num_actions_=%d, policy_body_dim_=%d, policy_obs_dim_=%d, "
             "obs_history_len_=%d, inference=%.0f Hz, gaitStyleMode=%s",
             name_.c_str(), num_actions_, policy_body_dim_, policy_obs_dim_, obs_history_len_,
             inference_frequency_, gait_style_mode_.c_str());
    return true;
  }

  void MoREController::initHistoryBuffer()
  {
    history_frames_.clear();
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(policy_body_dim_);
    for (int i = 0; i < obs_history_len_; ++i)
    {
      history_frames_.push_back(zero);
    }
  }

  void MoREController::packHistoryTensor()
  {
    // mjlab runner at act(obs_t, history): history[-1] == obs[:, num_gait:] (current body obs).
    // After each env.step: history = cat(history[:, 1:], body_{t+1}).
    // Pack as cat(stored_frames[1:], singleInputData_) — NOT stored_frames alone.
    int row = 0;
    const int n = static_cast<int>(history_frames_.size());
    const int past_start = (n > 1) ? 1 : 0;
    for (int i = past_start; i < n; ++i)
    {
      history_tensor_.segment(row, policy_body_dim_) = history_frames_[i].cast<float>();
      row += policy_body_dim_;
    }
    if (row + policy_body_dim_ <= history_tensor_.size())
    {
      history_tensor_.segment(row, policy_body_dim_) = singleInputData_.cast<float>();
    }
  }

  void MoREController::commitHistoryFrame()
  {
    if (history_frames_.empty())
    {
      return;
    }
    history_frames_.pop_front();
    history_frames_.push_back(singleInputData_);
    packHistoryTensor();
  }

  void MoREController::logPolicyObservation(const Eigen::VectorXd& motion_style_weights,
                                           const Eigen::Vector3d& velocity_commands,
                                           const Eigen::Vector2d& posture_commands) const
  {
    if (!log_policy_obs_)
    {
      return;
    }

    int style_idx = -1;
    for (int i = 0; i < motion_style_weights.size(); ++i)
    {
      if (motion_style_weights[i] > 0.5)
      {
        style_idx = i;
        break;
      }
    }
    const char* style_name = motionStyleHumanName(style_idx);
    const std::string one_hot = motionStyleOneHotString(style_idx, num_gait_);

    const double throttle = log_policy_obs_throttle_sec_ > 0.0 ? log_policy_obs_throttle_sec_ : 2.0;
    ROS_INFO_THROTTLE(
        throttle,
        "[%s] obs style=%d %s cmd_vel=[%.3f,%.3f,%.3f] posture=[%.3f,%.3f] dims(body=%d,policy=%d)",
        name_.c_str(), style_idx + 1, style_name,
        velocity_commands.x(), velocity_commands.y(), velocity_commands.z(),
        posture_commands.x(), posture_commands.y(), policy_body_dim_, policy_obs_dim_);

    if (!ros_logger_)
    {
      return;
    }

    ros_logger_->publishVector("/rl_controller/more_obs/style_weights", motion_style_weights);
    ros_logger_->publishValue("/rl_controller/more_obs/style_index",
                              static_cast<double>(style_idx < 0 ? -1 : style_idx));
    Eigen::VectorXd vel_cmd(3);
    vel_cmd << velocity_commands.x(), velocity_commands.y(), velocity_commands.z();
    ros_logger_->publishVector("/rl_controller/more_obs/velocity_commands", vel_cmd);
    ros_logger_->publishVector("/rl_controller/more_obs/posture_commands", posture_commands);

    int index = 0;
    for (const auto& key : singleInputDataKeys_)
    {
      const auto& meta = singleInputDataID_.at(key);
      const int dim = static_cast<int>(meta[1]);
      ros_logger_->publishVector("/rl_controller/more_obs/" + key,
                                  singleInputData_.segment(index, dim));
      index += dim;
    }
    const Eigen::VectorXd history_flat = history_tensor_.cast<double>();
    ros_logger_->publishVector("/rl_controller/more_obs/history_flat", history_flat);
  }

  void MoREController::styleCommandCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    if (!msg)
    {
      return;
    }
    const int idx = std::max(0, std::min(static_cast<int>(msg->data), num_gait_ - 1));
    bool style_changed = false;
    {
      std::lock_guard<std::mutex> lock(style_command_mutex_);
      style_changed = style_command_index_ != idx;
      style_command_index_ = idx;
    }
    if (style_changed)
    {
      ROS_INFO("[%s] motion style command -> 风格%d index=%d one_hot=%s",
               name_.c_str(), idx + 1, idx, motionStyleOneHotString(idx, num_gait_).c_str());
      syncArmControllerMode(idx);
    }
  }

  void MoREController::postureResetControlCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    if (!msg)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(posture_reset_mutex_);
    if (!msg->data)
    {
      // 与 reset 请求共用同一私有 topic，保证快速退出/重进时按发布顺序释放。
      posture_reset_pending_ = false;
      posture_reset_owned_ = false;
      if (gait_receiver_)
      {
        gait_receiver_->clearPostureTargetOverride();
      }
      return;
    }

    if (!posture_reset_accepting_ || !gait_receiver_ || posture_reset_owned_)
    {
      return;
    }

    if (!gait_receiver_->setPostureTargetOverride(Eigen::Vector2d::Zero()))
    {
      // pause/切出期间拒绝事件，避免把陈旧复位带到下一次 resume。
      return;
    }

    posture_reset_owned_ = true;
    posture_reset_pending_ = true;
    ROS_INFO("[%s] MoRE posture reset requested; tracking height/pitch target [0, 0]",
             name_.c_str());
  }

  int MoREController::getCurrentGaitStyleIndex() const
  {
    // 动作播放期间锁定风格，避免 auto 模式被 Python reset 阶段 state=1 误导返回 0
    // （包括 style0 也需要锁，否则 cmdStance 变化会导致 style 漂移）
    // restore_time_ 非零表示已执行风格恢复，应立即解锁
    if (action_pending_restore_ && restore_time_.isZero())
    {
      return pre_action_style_;
    }
    if (!gait_receiver_)
    {
      return default_gait_style_index_;
    }
    const CommandDataRL cmd = gait_receiver_->getCurrentCommand();
    const Eigen::VectorXd weights = computeMotionStyleWeights(cmd);
    for (int i = 0; i < weights.size(); ++i)
    {
      if (weights[i] > 0.5)
      {
        return i;
      }
    }
    return default_gait_style_index_;
  }

  bool MoREController::hasExternalArmCommand() const
  {
    return arm_command_replacement_enabled_ && arm_controller_ != nullptr &&
           arm_controller_->hasExternalTarget();
  }

  bool MoREController::hasExternalWaistCommand() const
  {
    return waist_command_replacement_enabled_ && waist_controller_ != nullptr &&
           waist_controller_->hasExternalTarget();
  }

  int MoREController::resolveUnfrozenArmControlMode(int gait_style_index) const
  {
    if (gait_style_index == 0)
    {
      // mode0 仅是冻结派生状态，不能成为长期偏好；任何非2值安全回落到 mode1。
      return pose_arm_control_mode_ == 2 ? 2 : 1;
    }
    if (usesThreeGaitExperts() && gait_style_index == 2)
    {
      return 2;
    }
    // 二风格 walk 与三风格 walk_policy 均走策略手臂
    return 1;
  }

  int MoREController::resolveArmControlMode(int gait_style_index) const
  {
    const int unfrozen_mode = resolveUnfrozenArmControlMode(gait_style_index);
    // 冻结属于外部手臂(mode2)的派生状态；基础 mode1 永远不受冻结影响。
    return (arm_frozen_ && unfrozen_mode == 2) ? 0 : unfrozen_mode;
  }

  int MoREController::resolveWaistControlMode(int gait_style_index) const
  {
    if (gait_style_index == 0)
    {
      return pose_waist_control_mode_;
    }
    if (usesThreeGaitExperts() && gait_style_index == 2)
    {
      return 2;
    }
    return 1;
  }

  bool MoREController::requestArmControlMode(int target_mode)
  {
    if (!arm_controller_)
    {
      return false;
    }

    // mode0 仍按原有全局语义直接处理；它不能反写为 pose/style 的长期偏好。
    // 动作播放期间的临时模式也由原动作状态机保存和恢复，不污染用户偏好。
    if ((target_mode != 1 && target_mode != 2) || action_pending_restore_)
    {
      return arm_controller_->changeMode(target_mode);
    }

    const int current_style = getCurrentGaitStyleIndex();

    // 双 Trigger 与 X+A 最终都会形成明确的全局 mode1/mode2 请求。
    // 这里以该明确意图同步 MoRE 的站立和下一次行走偏好，避免再从
    // ArmController 的瞬时/过渡状态反推用户意图。
    pose_arm_control_mode_ = target_mode;
    stand_origin_style_ = target_mode;

    int target_style = current_style;
    if (current_style != 0 && usesThreeGaitExperts())
    {
      target_style = target_mode;
      if (gait_style_mode_ == "command")
      {
        std::lock_guard<std::mutex> lock(style_command_mutex_);
        style_command_index_ = target_mode;
      }
    }

    // syncArmControllerMode 会在进入基础 mode1 时清除冻结；基础 mode2 下
    // 保留 X+B 冻结，只有 X+B 解冻或切入 mode1 才解除。
    syncArmControllerMode(target_style);
    ROS_INFO("[%s] Global arm mode intent -> %d (style %d -> %d, pose=%d, stand_origin=%d, frozen=%d)",
             name_.c_str(), target_mode, current_style, target_style,
             pose_arm_control_mode_, stand_origin_style_, arm_frozen_);
    return true;
  }

  void MoREController::handleQuestArmControlButtons(const kuavo_msgs::JoySticks& joy)
  {
    const bool arm_active = arm_controller_ && arm_command_replacement_enabled_;
    const bool waist_active = waist_controller_ && waist_command_replacement_enabled_;
    if (!arm_active && !waist_active)
    {
      return;
    }
    // X+A 由 QuestControlFSM 作为唯一入口，通过全局 mode 请求进入
    // requestArmControlMode()；此处只独占处理 X+B，避免同一按键被两处翻转。
    // X+B 只冻结外部手臂：基础 mode2 可切换 mode0↔mode2，基础 mode1 忽略。
    if (arm_active &&
        joy.left_first_button_pressed && !quest_joystick_prev_.right_second_button_pressed &&
        joy.right_second_button_pressed)
    {
      const int current_style = getCurrentGaitStyleIndex();
      const int unfrozen_mode = resolveUnfrozenArmControlMode(current_style);
      if (unfrozen_mode == 2)
      {
        arm_frozen_ = !arm_frozen_;
        if (arm_frozen_)
        {
          // 冻结: 切 mode=0 (kLocked)。kLocked 下 jointStateCallback 丢弃 /kuavo_arm_traj，
          // 冻结完全独立，不受 Python IK hold_arm_timer 影响。
          arm_controller_->changeMode(0);
        }
        else
        {
          arm_controller_->changeMode(unfrozen_mode);
        }
        ROS_INFO("[%s] VR X+B: arm %s (style=%d)", name_.c_str(),
                 arm_frozen_ ? "FROZEN" : "UNFROZEN", current_style);
      }
      else
      {
        ROS_INFO("[%s] VR X+B ignored: unfrozen arm mode=%d (style=%d)",
                 name_.c_str(), unfrozen_mode, current_style);
      }
    }
  }

  void MoREController::questJoystickCallback(const kuavo_msgs::JoySticks::ConstPtr& msg)
  {
    if (!msg)
    {
      return;
    }
    if (!quest_joystick_initialized_)
    {
      quest_joystick_prev_ = *msg;
      quest_joystick_initialized_ = true;
      return;
    }

    handleQuestArmControlButtons(*msg);
    quest_joystick_prev_ = *msg;
  }

  bool MoREController::isArmControllerActive() const
  {
    return arm_command_replacement_enabled_ && arm_controller_ != nullptr;
  }

  void MoREController::syncWaistControllerMode(int gait_style_index, bool style_changed)
  {
    if (!waist_controller_)
    {
      return;
    }
    const int waist_mode = resolveWaistControlMode(gait_style_index);
    const int current_mode = waist_controller_->getMode();

    if (style_changed)
    {
      if (current_mode != waist_mode)
      {
        waist_controller_->changeMode(waist_mode);
        ROS_INFO("[%s] WaistController mode -> %d (gait style %d -> %d, more_mode_=%d, pose_waist_mode=%d)",
                 name_.c_str(), waist_mode, last_synced_gait_style_index_, gait_style_index, more_mode_,
                 pose_waist_control_mode_);
      }
      return;
    }

    // walk 风格(1/2)：策略/外部腰部；pose 风格尊重 VR X+A 设定的模式
    if (gait_style_index != 0 && current_mode != waist_mode)
    {
      waist_controller_->changeMode(waist_mode);
      ROS_INFO("[%s] WaistController mode -> %d (walk more_mode_=%d)", name_.c_str(), waist_mode, more_mode_);
    }
  }

  void MoREController::syncArmControllerMode(int gait_style_index)
  {
    const bool style_changed = (gait_style_index != last_synced_gait_style_index_);
    syncWaistControllerMode(gait_style_index, style_changed);

    const int unfrozen_arm_mode = resolveUnfrozenArmControlMode(gait_style_index);
    // 一旦目标不再是外部手臂 mode2，就永久取消冻结；mode2 状态之间切换则保留。
    if (unfrozen_arm_mode != 2 && arm_frozen_)
    {
      arm_frozen_ = false;
      ROS_INFO("[%s] Arm freeze cleared: unfrozen arm mode -> %d (gait_style=%d)",
               name_.c_str(), unfrozen_arm_mode, gait_style_index);
    }

    if (!arm_controller_)
    {
      if (style_changed)
      {
        last_synced_gait_style_index_ = gait_style_index;
      }
      return;
    }
    const int arm_mode = resolveArmControlMode(gait_style_index);
    const int requested_mode = arm_controller_->getRequestedMode();

    if (style_changed)
    {
      if (requested_mode != arm_mode)
      {
        arm_controller_->changeMode(arm_mode);
        ROS_INFO("[%s] ArmController mode -> %d (gait style %d -> %d, more_mode_=%d, pose_arm_mode=%d)",
                 name_.c_str(), arm_mode, last_synced_gait_style_index_, gait_style_index, more_mode_,
                 pose_arm_control_mode_);
      }
      last_synced_gait_style_index_ = gait_style_index;
      return;
    }

    // 所有风格（含 pose）：手臂模式被外部改错时纠正回来
    if (requested_mode != arm_mode)
    {
      arm_controller_->changeMode(arm_mode);
      ROS_INFO("[%s] ArmController mode -> %d (sync, gait_style=%d, more_mode_=%d)",
               name_.c_str(), arm_mode, gait_style_index, more_mode_);
    }
  }

  int MoREController::resolveMotionStyleIndex(const CommandDataRL& cmd) const
  {
    // 动作期间锁定行走风格，避免 auto 模式被 Python reset 阶段 state=1 误导
    // 包括 style0 也需要锁，否则 cmdStance 变化会导致 policy one-hot 漂移
    // restore_time_ 非零表示已执行风格恢复，应立即解锁让正常逻辑接管风格判定
    if (action_pending_restore_ && restore_time_.isZero())
    {
      // command 模式：尊重 callback 显式设置的 style_command_index_（style1→2 等）
      // auto 模式：锁定到 pre_action_style_，防止 cmdStance 变化导致风格漂移
      if (gait_style_mode_ == "command")
      {
        std::lock_guard<std::mutex> lock(style_command_mutex_);
        return style_command_index_;
      }
      // style 1 (walk_policy_arm) 训练时有摆臂，动作期间手臂轨迹与训练分布不一致；
      // 映射到 style 2 (walk_external_arm) 让策略走外部手臂 expert，保持步态稳定
      return (pre_action_style_ == 1) ? 2 : pre_action_style_;
    }

    int style_idx = default_gait_style_index_;
    if (gait_style_mode_ == "command")
    {
      std::lock_guard<std::mutex> lock(style_command_mutex_);
      style_idx = style_command_index_;
    }
    else  // auto: 由 cmd_stance 和 stand_origin_style_ 决定风格
    {
      const bool is_standing = (cmd.cmdStance_ >= 0.5);
      if (is_standing)
      {
        style_idx = 0;  // pose（手臂永远外部，mode2/mode3 由 stand_origin_style_ 区分）
      }
      // else if (usesThreeGaitExperts() && hasExternalArmCommand())  // deprecated: VR自动切
      // {
      //   style_idx = 2;
      // }
      else
      {
        style_idx = stand_origin_style_;  // X+A 手动切换的行走风格
      }
    }
    return std::max(0, std::min(style_idx, num_gait_ - 1));
  }

  Eigen::VectorXd MoREController::computeMotionStyleWeights(const CommandDataRL& cmd) const
  {
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(num_gait_);
    const int style_idx = resolveMotionStyleIndex(cmd);
    weights(style_idx) = 1.0;
    return weights;
  }

  Eigen::Vector2d MoREController::computePostureCommands(int gait_style_index)
  {
    // 从行走回到站立时，清 squat 状态恢复站姿
    if (last_posture_style_index_ >= 0 && last_posture_style_index_ != gait_style_index && gait_style_index == 0)
    {
      squat_blend_target_ = 0.0;
      smoothed_squat_ = 0.0;
      if (gait_receiver_)
      {
        gait_receiver_->resetPostureCommand();
      }
      ROS_INFO("[%s] squat state cleared (style %d -> 0)", name_.c_str(), last_posture_style_index_);
    }
    last_posture_style_index_ = gait_style_index;

    // [squat_height_delta_m, trunk_pitch_delta_rad]
    Eigen::Vector2d posture_commands = Eigen::Vector2d::Zero();
    if (gait_receiver_)
    {
      posture_commands = gait_receiver_->getCurrentPostureCommand();
    }

    // 仅由 MoRE 专属退出事件触发收尾；普通 /cmd_pose 数值回零不再被误判。
    // override 在到零后仍保持，直到下一次私有控制消息 false（新会话），
    // 保证退出前尾帧无法重新覆盖默认站姿。
    {
      std::lock_guard<std::mutex> lock(posture_reset_mutex_);
      const bool posture_at_zero =
          std::max(std::abs(posture_commands.x()), std::abs(posture_commands.y())) < 1e-4;
      if (posture_reset_pending_ && posture_at_zero)
      {
        squat_blend_target_ = 0.0;
        squat_blend_start_ = ros::Time::now().toSec();
        if (!posture_smooth_enabled_)
        {
          smoothed_squat_ = 0.0;
        }
        if (gait_receiver_)
        {
          // 只清数值，不释放 override；ownership 由下一次私有 false 消息释放。
          gait_receiver_->resetPostureCommand();
        }
        posture_commands.setZero();
        if (waist_controller_ && waist_controller_->getMode() == 2)
        {
          waist_controller_->changeMode(1);
          waist_controller_->changeMode(2);
        }
        posture_reset_pending_ = false;
        ROS_INFO("[%s] MoRE posture reset reached zero; squat state cleared + waist reset",
                 name_.c_str());
      }
    }

    posture_commands.x() = std::clamp(posture_commands.x(), posture_height_min_, posture_height_max_);
    posture_commands.y() = std::clamp(posture_commands.y(), posture_bend_min_, posture_bend_max_);

    // 下蹲/起身 smoothstep 平滑过渡：目标值变化时从当前平滑值出发追新目标
    if (posture_smooth_enabled_)
    {
      const double raw = posture_commands.x();
      if (std::abs(raw - squat_blend_target_) > 1e-6)
      {
        squat_blend_start_ = ros::Time::now().toSec();
        squat_blend_target_ = raw;
      }
      const double elapsed = ros::Time::now().toSec() - squat_blend_start_;
      const double t = std::min(elapsed / posture_smooth_duration_, 1.0);
      const double alpha = t * t * (3.0 - 2.0 * t);  // smoothstep
      smoothed_squat_ += alpha * (squat_blend_target_ - smoothed_squat_);
      if (t >= 1.0)
      {
        smoothed_squat_ = squat_blend_target_;
      }
      posture_commands.x() = smoothed_squat_;
    }

    if (posture_nonzero_gait_style_index_ >= 0 && gait_style_index != posture_nonzero_gait_style_index_)
    {
      posture_commands.setZero();
    }
    return posture_commands;
  }

  void MoREController::reset()
  {
    actions_.setZero();
    singleInputData_.setZero();
    policy_obs_.setZero();
    {
      std::lock_guard<std::mutex> lock(style_command_mutex_);
      style_command_index_ = default_gait_style_index_;
    }
    initHistoryBuffer();
    history_tensor_.setZero();
    my_yaw_offset_ = 0.0;
    yaw_offset_initialized_ = false;
    pose_arm_control_mode_ = 1;  // 与 ArmController::reset() 的默认 mode1 保持一致
    // pose_waist_control_mode_ = 1;
    stand_origin_style_ = 1;  // 初始默认为 style1 侧
    last_synced_gait_style_index_ = -1;
    quest_joystick_initialized_ = false;
    arm_frozen_ = false;
    if (arm_controller_)
    {
      arm_controller_->reset();
    }
    arm_takeover_blender_.reset();
    last_stance_state_for_blend_ = false;
    smoothed_squat_ = 0.0;
    squat_blend_target_ = 0.0;
    last_posture_style_index_ = -1;
    {
      std::lock_guard<std::mutex> lock(posture_reset_mutex_);
      posture_reset_pending_ = false;
      posture_reset_owned_ = false;
      if (gait_receiver_)
      {
        gait_receiver_->clearPostureTargetOverride();
        gait_receiver_->resetPostureCommand();
      }
    }
    if (waist_controller_)
    {
      waist_controller_->reset();
    }
    sensor_data_updated_ = false;
    ROS_INFO("[%s] reset", name_.c_str());
  }

  void MoREController::pause()
  {
    {
      std::lock_guard<std::mutex> lock(posture_reset_mutex_);
      RLControllerBase::pause();
      posture_reset_accepting_ = false;
      posture_reset_pending_ = false;
      posture_reset_owned_ = false;
      if (gait_receiver_)
      {
        gait_receiver_->clearPostureTargetOverride();
        gait_receiver_->setEnabled(false);
      }
    }

    // Ruiwo 手臂增益已通过 joint_cmd 实时下发，无需单独调用 ROS 服务切换
  }

  void MoREController::resume()
  {
    // 在控制器仍为 PAUSED 时完成全部重置，避免推理线程看到半重置状态。
    {
      std::lock_guard<std::mutex> lock(posture_reset_mutex_);
      posture_reset_accepting_ = false;
    }
    reset();
    if (gait_receiver_)
    {
      gait_receiver_->resetToStance();
      gait_receiver_->setEnabled(true);
    }
    {
      std::lock_guard<std::mutex> lock(posture_reset_mutex_);
      RLControllerBase::resume();
      posture_reset_accepting_ = (state_ == ControllerState::RUNNING);
      if (!posture_reset_accepting_ && gait_receiver_)
      {
        gait_receiver_->setEnabled(false);
      }
    }

    // Ruiwo 手臂增益已通过 motor_pdo_kp/kd（skw_rl_param.info）在 joint_cmd 中实时下发

    ROS_INFO("[%s] Controller resumed, reset state", name_.c_str());
  }

  bool MoREController::requestToExit() const
  {
    if (!sensor_data_updated_)
    {
      return false;
    }

    const Eigen::VectorXd state = getRobotState();
    if (state.size() < 12)
    {
      return false;
    }

    const double roll = state(2);
    const double pitch = state(1);
    const double roll_deg = std::abs(roll) * 180.0 / M_PI;
    const double pitch_deg = std::abs(pitch) * 180.0 / M_PI;

    const double fall_threshold_deg = 60.0;
    const bool is_fallen = (roll_deg > fall_threshold_deg) || (pitch_deg > fall_threshold_deg);

    if (is_fallen)
    {
      ROS_WARN_THROTTLE(1.0, "[%s] Detected fall: roll=%.2f deg, pitch=%.2f deg, requesting exit",
                        name_.c_str(), roll_deg, pitch_deg);
    }

    return is_fallen;
  }

  bool MoREController::isAllowToExit() const
  {
    if (!gait_receiver_)
    {
      return true;
    }
    return gait_receiver_->getCurrentCommand().cmdStance_ >= 0.5;
  }

  bool MoREController::shouldRunInference() const
  {
    if (state_ != ControllerState::RUNNING || !gait_receiver_)
    {
      return false;
    }
    return RLControllerBase::shouldRunInference();
  }

  void MoREController::updateObservation(const Eigen::VectorXd& state_est,
                                         const SensorData& sensor_data)
  {
    if (!gait_receiver_)
    {
      return;
    }

    const CommandDataRL cmd = gait_receiver_->getCurrentCommand();

    if (!yaw_offset_initialized_)
    {
      const auto mat = sensor_data.quat_.toRotationMatrix();
      const double current_yaw = std::atan2(mat(1, 2), mat(0, 2));
      my_yaw_offset_ = -current_yaw;
      while (my_yaw_offset_ > M_PI)
        my_yaw_offset_ -= 2 * M_PI;
      while (my_yaw_offset_ < -M_PI)
        my_yaw_offset_ += 2 * M_PI;
      yaw_offset_initialized_ = true;
      ROS_INFO("[%s] Initialized yaw_offset: %.4f rad", name_.c_str(), my_yaw_offset_);
    }

    // mjlab ``generated_commands(twist)`` feeds physical body-frame m/s into obs — NOT
    // Kuavo skw ``commandData.scale`` (×1.6, ×2, …). Scaling here was collapsing deploy I/O.
    double vx = cmd.cmdVelLineX_;
    if (vx < 0.0)
    {
      vx *= cmdVelLineXNegScale_;
    }
    Eigen::Vector3d velocity_commands(vx, cmd.cmdVelLineY_, cmd.cmdVelAngularZ_);

    if (yaw_compensation_enabled_)
    {
      const double angular_z = velocity_commands(2);
      const double linear_xy =
          std::hypot(velocity_commands(0), velocity_commands(1));
      // 仅原地转弯（几乎无线速度）时注入 x 偏置；有前进/侧移命令时不修改，避免“有角速度就不往前走”
      if (std::abs(angular_z) > yaw_compensation_threshold_ &&
          linear_xy < yaw_compensation_x_velocity_threshold_)
      {
        double x_bias = yaw_compensation_x_bias_;
        if (yaw_compensation_separate_enabled_)
        {
          x_bias = (angular_z > 0) ? yaw_compensation_x_bias_counterclockwise_
                                   : yaw_compensation_x_bias_clockwise_;
        }
        velocity_commands(0) += x_bias;
      }
    }

    const Eigen::VectorXd motion_style_weights = computeMotionStyleWeights(cmd);
    int gait_style_index = default_gait_style_index_;
    for (int i = 0; i < motion_style_weights.size(); ++i)
    {
      if (motion_style_weights[i] > 0.5)
      {
        gait_style_index = i;
        break;
      }
    }
    const Eigen::Vector2d posture_commands = computePostureCommands(gait_style_index);

    Eigen::VectorXd jointPos = sensor_data.jointPos_ - defalutJointPosRL_;
    const Eigen::VectorXd& jointVel = sensor_data.jointVel_;
    const Eigen::Vector3d bodyAngVel = sensor_data.angularVel_;

    const auto quat_offset = Eigen::AngleAxisd(-my_yaw_offset_, Eigen::Vector3d::UnitZ()) * sensor_data.quat_;
    const Eigen::Matrix3d R = quat_offset.matrix();
    const Eigen::Vector3d gravity_world(0, 0, -1);
    const Eigen::Vector3d projected_gravity = R.transpose() * gravity_world;

    const Eigen::VectorXd local_action = getCurrentAction();

    const Eigen::VectorXd scalar_command =
        Eigen::VectorXd::Constant(1, 1.0 - static_cast<double>(cmd.cmdStance_));

    const std::map<std::string, Eigen::VectorXd> singleInputDataMap = {
        {"velocity_commands", velocity_commands},
        {"scalar_command", scalar_command},
        {"posture_commands", posture_commands},
        {"bodyAngVel", bodyAngVel},
        {"projected_gravity", projected_gravity},
        {"gravity_body", projected_gravity},
        {"joint_pos", jointPos},
        {"jointPos", jointPos},
        {"joint_vel", jointVel},
        {"jointVel", jointVel},
        {"actions", local_action},
        {"action", local_action},
    };

    if (!singleInputDataKeys_.empty())
    {
      int index = 0;
      for (const auto& key : singleInputDataKeys_)
      {
        const auto& value = singleInputDataID_.at(key);
        singleInputData_.segment(index, static_cast<int>(value[1])) =
            singleInputDataMap.at(key).segment(static_cast<int>(value[0]), static_cast<int>(value[1])) * value[2];
        index += static_cast<int>(value[1]);
      }
    }

    policy_obs_.head(num_gait_) = motion_style_weights;
    policy_obs_.tail(policy_body_dim_) = singleInputData_;

    // history 最后一帧 = 当前 body obs（与 mjlab act 一致）；commit 在 inference 成功后
    packHistoryTensor();

    logPolicyObservation(motion_style_weights, velocity_commands, posture_commands);

    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/more_policy_obs", policy_obs_);
      ros_logger_->publishVector("/rl_controller/more_body_obs", singleInputData_);
      ros_logger_->publishVector("/rl_controller/more_gait_weights", motion_style_weights);
    }
  }

  bool MoREController::inference(const Eigen::VectorXd& /*observation*/, Eigen::VectorXd& action)
  {
    try
    {
      infer_request_ = compiled_model_.create_infer_request();
      const auto inputs = compiled_model_.inputs();

      onnx_obs_buffer_ = policy_obs_.cast<float>();
      const ov::Shape obs_shape = inputs[0].get_shape();
      const size_t expected_obs_elems = ovShapeElemCount(obs_shape);
      if (static_cast<size_t>(onnx_obs_buffer_.size()) != expected_obs_elems)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] observations size %ld vs ONNX %zu", name_.c_str(),
                           static_cast<long>(onnx_obs_buffer_.size()), expected_obs_elems);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }
      ov::Tensor obs_tensor(inputs[0].get_element_type(), obs_shape, onnx_obs_buffer_.data());
      infer_request_.set_input_tensor(0, obs_tensor);

      if (has_obs_history_encoder_)
      {
        if (inputs.size() < 2)
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] has_obs_history_encoder_=true but ONNX has %zu inputs",
                             name_.c_str(), inputs.size());
          action = Eigen::VectorXd::Zero(num_actions_);
          return false;
        }
        const ov::Shape hist_shape = inputs[1].get_shape();
        const size_t expected_hist_elems = ovShapeElemCount(hist_shape);
        if (static_cast<size_t>(history_tensor_.size()) != expected_hist_elems)
        {
          ROS_ERROR_THROTTLE(1.0, "[%s] history size %ld vs ONNX %zu", name_.c_str(),
                             static_cast<long>(history_tensor_.size()), expected_hist_elems);
          action = Eigen::VectorXd::Zero(num_actions_);
          return false;
        }
        ov::Tensor hist_tensor(inputs[1].get_element_type(), hist_shape, history_tensor_.data());
        infer_request_.set_input_tensor(1, hist_tensor);
      }

      infer_request_.start_async();
      infer_request_.wait();

      const auto output_tensor = infer_request_.get_output_tensor(0);
      const size_t output_buf_length = output_tensor.get_size();
      const auto* output_buf = output_tensor.data<float>();
      const size_t expected_output_length =
          withArm_ ? static_cast<size_t>(jointNum_ + jointArmNum_ + waistNum_)
                  : static_cast<size_t>(jointNum_ + waistNum_);

      if (output_buf_length != expected_output_length)
      {
        ROS_ERROR_THROTTLE(1.0, "[%s] output size %zu vs expected %zu",
                           name_.c_str(), output_buf_length, expected_output_length);
        action = Eigen::VectorXd::Zero(num_actions_);
        return false;
      }

      action.resize(static_cast<int>(output_buf_length));
      for (int i = 0; i < static_cast<int>(output_buf_length); ++i)
      {
        action[i] = output_buf[i];
      }
      clip(action, clipActions_);
      commitHistoryFrame();
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] inference failed: %s", name_.c_str(), e.what());
      action = Eigen::VectorXd::Zero(num_actions_);
      return false;
    }
  }

  Eigen::VectorXd MoREController::updateRLcmd(const Eigen::VectorXd& /*measuredRbdState*/)
  {
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
    else if (getCurrentGaitStyleIndex() != 1)
    {
      // 仅 walk_policy_arm(1) 使用策略手臂+腰部；pose(0)/walk_external_arm(2) 走标称/外部
      if (is_roban_)
      {
        local_action.segment(0, waistNum_).setZero();                        // 腰部
        local_action.segment(waistNum_ + jointNum_, jointArmNum_).setZero(); // 手臂
      }
      else
      {
        local_action.segment(jointNum_, waistNum_).setZero();                 // 腰部
        local_action.segment(jointNum_ + waistNum_, jointArmNum_).setZero(); // 手臂
      }
    }

    // 站立外部手臂到行走RL摆臂的平滑接管（与 AMP armRlTakeoverBlend 一致）
    {
      CommandDataRL cmd_data = gait_receiver_->getPolicyCommand();
      cmd_data.scale();
      bool is_standing = (cmd_data.cmdStance_ >= 1.0);
      applyArmTakeoverBlend(local_action, ros::Time::now(), is_standing);
    }

    Eigen::VectorXd jointTor(jointNum_ + jointArmNum_ + waistNum_);

    if (is_roban_)
    {
      motorPos.segment(waistNum_, jointNum_) =
          ankleSolver_.joint_to_motor_position(jointPos.segment(waistNum_, jointNum_));
      motorVel.segment(waistNum_, jointNum_) = ankleSolver_.joint_to_motor_velocity(
          jointPos.segment(waistNum_, jointNum_), motorPos.segment(waistNum_, jointNum_),
          jointVel.segment(waistNum_, jointNum_));
      jointTor = -(jointKdRL_.cwiseProduct(motorVel));
      jointTor.segment(waistNum_, jointNum_) = ankleSolver_.motor_to_joint_torque(
          jointPos.segment(waistNum_, jointNum_), motorPos.segment(waistNum_, jointNum_),
          jointTor.segment(waistNum_, jointNum_));
    }
    else
    {
      motorPos.head(jointNum_) = ankleSolver_.joint_to_motor_position(jointPos.head(jointNum_));
      motorVel.head(jointNum_) = ankleSolver_.joint_to_motor_velocity(
          jointPos.head(jointNum_), motorPos.head(jointNum_), jointVel.head(jointNum_));
      jointTor = -(jointKdRL_.cwiseProduct(motorVel));
      jointTor.head(jointNum_) = ankleSolver_.motor_to_joint_torque(
          jointPos.head(jointNum_), motorPos.head(jointNum_), jointTor.head(jointNum_));
    }

    Eigen::VectorXd cmd(jointNum_ + jointArmNum_ + waistNum_);
    // CSP (mode 2): policy PD 力矩叠到 jointTor，与 AmpWalkController 一致（此前 mode2 仅阻尼 → 几乎无控制）
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; ++i)
    {
      jointTor[i] += jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] +
                                      defalutJointPosRL_[i]);
    }

    if (is_real_)
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; ++i)
      {
        if (JointControlModeRL_(i) == 0)
        {
          if (JointPDModeRL_(i) == 0)
          {
            cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] +
                                      defalutJointPosRL_[i]) -
                     jointKdRL_[i] * jointVel[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
          }
          else
          {
            cmd[i] = local_action[i] * actionScale_ * actionScaleTestRL_[i] + defalutJointPosRL_[i];
          }
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] +
                                    defalutJointPosRL_[i]);
        }
      }
    }
    else
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; ++i)
      {
        if (JointControlModeRL_(i) == 0)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] +
                                    defalutJointPosRL_[i]) -
                   jointKdRL_[i] * jointVel[i];
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointTor[i];
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
      }
    }

    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/cmd", cmd);
    }

    if (use_jointcmd_filter_)
    {
      const Eigen::VectorXd cmd_filter = jointCmdFilter_.update(cmd);
      Eigen::VectorXd filterState_full = Eigen::VectorXd::Zero(jointNum_ + jointArmNum_ + waistNum_);
      filterState_full.head(jointCmdFilterState_.size()) = jointCmdFilterState_;
      const Eigen::VectorXd actuation = cmd_filter.cwiseProduct(filterState_full) +
                                        cmd.cwiseProduct(Eigen::VectorXd::Ones(jointNum_ + jointArmNum_ + waistNum_) -
                                                         filterState_full);
      if (ros_logger_)
      {
        ros_logger_->publishVector("/rl_controller/actuation", actuation);
      }
      return actuation;
    }

    if (ros_logger_)
    {
      ros_logger_->publishVector("/rl_controller/actuation", cmd);
    }
    return cmd;
  }

  void MoREController::applyArmTakeoverBlend(Eigen::VectorXd& action, const ros::Time& time, bool is_standing)
  {
    if (!arm_rl_takeover_blend_enabled_ || jointArmNum_ <= 0 || !arm_command_replacement_enabled_)
    {
      return;
    }

    if (action.size() < jointNum_ + waistNum_ + jointArmNum_)
    {
      return;
    }

    const int arm_start = jointNum_ + waistNum_;

    // 站立状态：根据配置置零手臂 action，并重置 blender
    if (is_standing)
    {
      if (arm_zero_action_in_standing_)
      {
        action.segment(arm_start, jointArmNum_).setZero();
      }
      arm_takeover_blender_.reset();
      last_stance_state_for_blend_ = true;
      return;
    }

    // 行走状态
    // 检测站立→行走切换，启动平滑过渡
    if (last_stance_state_for_blend_)
    {
      arm_takeover_blender_.start(time.toSec(), jointArmNum_);
    }
    last_stance_state_for_blend_ = false;

    // 应用平滑混合
    if (arm_takeover_blender_.isActive())
    {
      action.segment(arm_start, jointArmNum_) =
          arm_takeover_blender_.blendArmAction(time.toSec(), action.segment(arm_start, jointArmNum_));
    }
  }

  void MoREController::actionToJointCmd(const Eigen::VectorXd& actuation,
                                        const Eigen::VectorXd& /*measuredRbdState*/,
                                        kuavo_msgs::jointCmd& joint_cmd)
  {
    const int total_joints = jointNum_ + jointArmNum_ + waistNum_;
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
        joint_cmd.joint_q.push_back(defalutJointPosRL_[i]);
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
      const SensorData sensor_data = getRobotSensorData();
      const Eigen::VectorXd current_jointPos = sensor_data.jointPos_.head(total_joints);
      const Eigen::VectorXd current_jointVel = sensor_data.jointVel_.head(total_joints);
      for (int i = 0; i < total_joints; ++i)
      {
        if (JointControlModeRL_(i) == 0)
        {
          if (JointPDModeRL_(i) == 0)
          {
            joint_cmd.joint_q.push_back(0.0);
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(0);
            joint_cmd.joint_kd.push_back(0);
            joint_cmd.tau.push_back(actuation(i));
          }
          else
          {
            joint_cmd.joint_q.push_back(actuation(i));
            joint_cmd.joint_v.push_back(0.0);
            joint_cmd.joint_kp.push_back(jointKpRL_[i]);
            joint_cmd.joint_kd.push_back(jointKdRL_[i]);
            joint_cmd.tau.push_back(0.0);
          }
          joint_cmd.tau_ratio.push_back(1);
          joint_cmd.tau_max.push_back(torqueLimitsRL_[i]);
          joint_cmd.control_modes.push_back(JointControlModeRL_(i));
        }
        else
        {
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

  bool MoREController::updateImpl(const ros::Time& time,
                                  const SensorData& /*sensor_data*/,
                                  const Eigen::VectorXd& measuredRbdState,
                                  kuavo_msgs::jointCmd& joint_cmd)
  {
    gait_receiver_->update(time, baseStateRL_, feetPositionsRL_);
    // 这里只做「用当前 actions_ 计算 actuation，再映射到 joint_cmd」
    // 动作期间手臂值由 updateArmCommand 中 arm_controller_->update() 写入（
    //   Python 轨迹通过 /kuavo_arm_traj → ArmController 处理，最后写入 joint_cmd）
    const Eigen::VectorXd actuation = updateRLcmd(measuredRbdState);
    actionToJointCmd(actuation, measuredRbdState, joint_cmd);
    joint_cmd.header.stamp = time;
    return true;
  }

  void MoREController::preprocessSensorData(SensorData& sensor_data)
  {
    // 先执行基类中的通用滤波逻辑（RL IMU 滤波）
    RLControllerBase::preprocessSensorData(sensor_data);

    if (is_roban_)  // roban模型使用旧顺序训练
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

  void MoREController::initArmControl(const std::string& urdf_path)
  {
    if (arm_command_replacement_enabled_ && jointArmNum_ > 0)
    {
      try
      {
        arm_controller_ = std::make_unique<ArmController>(nh_, jointNum_, waistNum_, jointArmNum_, ros_logger_);

        Eigen::VectorXd arm_kp, arm_kd;
        if (is_roban_)
        {
          arm_kp = jointKpRL_.segment(waistNum_ + jointNum_, jointArmNum_);
          arm_kd = jointKdRL_.segment(waistNum_ + jointNum_, jointArmNum_);
        }
        else
        {
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

        Eigen::VectorXd default_arm_pos;
        if (defalutJointPosRL_.size() >= jointNum_ + waistNum_ + jointArmNum_)
        {
          if (is_roban_)
          {
            default_arm_pos = defalutJointPosRL_.segment(waistNum_ + jointNum_, jointArmNum_);
          }
          else
          {
            default_arm_pos = defalutJointPosRL_.segment(jointNum_ + waistNum_, jointArmNum_);
          }
        }
        else
        {
          default_arm_pos = Eigen::VectorXd::Zero(jointArmNum_);
          ROS_WARN("[%s] Cannot get default arm position, using zero vector", name_.c_str());
        }

        arm_controller_->loadSettings(arm_max_tracking_velocity_, arm_tracking_error_threshold_,
                                    arm_mode_interpolation_velocity_, default_arm_pos);

        syncArmControllerMode(style_command_index_);

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

  bool MoREController::updateArmCommand(const ros::Time& time,
                                        const SensorData& sensor_data,
                                        kuavo_msgs::jointCmd& joint_cmd)
  {
    if (!arm_command_replacement_enabled_ || jointArmNum_ == 0 || !arm_controller_)
    {
      return false;
    }

    // 动作播放后的超时清理
    checkAndRestoreAfterAction();

    // 先构建手-腿-腰完整关节数据，并同步 ArmController 内部状态
    // 确保后面的 syncArmControllerMode → applyModeChange 拿到的 current_arm_pos_
    // 是传感器真实值，而非 ArmController 内部陈旧的零值
    Eigen::VectorXd full_joint_pos(jointNum_ + waistNum_ + jointArmNum_);
    Eigen::VectorXd full_joint_vel(jointNum_ + waistNum_ + jointArmNum_);

    if (is_roban_)
    {
      const Eigen::VectorXd waist_pos = sensor_data.jointPos_.segment(0, waistNum_);
      const Eigen::VectorXd leg_pos = sensor_data.jointPos_.segment(waistNum_, jointNum_);
      const Eigen::VectorXd arm_pos = sensor_data.jointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
      const Eigen::VectorXd waist_vel = sensor_data.jointVel_.segment(0, waistNum_);
      const Eigen::VectorXd leg_vel = sensor_data.jointVel_.segment(waistNum_, jointNum_);
      const Eigen::VectorXd arm_vel = sensor_data.jointVel_.segment(waistNum_ + jointNum_, jointArmNum_);
      full_joint_pos << leg_pos, waist_pos, arm_pos;
      full_joint_vel << leg_vel, waist_vel, arm_vel;
    }
    else
    {
      full_joint_pos = sensor_data.jointPos_.head(jointNum_ + waistNum_ + jointArmNum_);
      full_joint_vel = sensor_data.jointVel_.head(jointNum_ + waistNum_ + jointArmNum_);
    }

    const size_t arm_start = jointNum_ + waistNum_;
    arm_controller_->updateInternalState(full_joint_pos, full_joint_vel, arm_start);

    // 动作活跃期（state==1, 恢复未启动）：锁定手臂为 mode 2，抵御外部干扰
    // 恢复已启动（restore_time_ 非零）：不再视作活跃，避免 Python reset 的 state=1 重新触发
    // 清理期：让 checkAndRestoreAfterAction 处理
    const bool action_active = (action_pending_restore_ && last_robot_action_state_ == 1
                                && restore_time_.isZero());
    const int arm_mode_before_sync = arm_controller_->getMode();
    if (action_active)
    {
      syncArmControllerMode(2);  // 活跃期所有 style 的动作都需要手臂 mode 2
      // VR 全程上锁，Python 轨迹通过 /kuavo_action_traj 独立通道进入 ArmController
    }
    else if (action_pending_restore_ && restore_time_.isZero())
    {
      // 清理期（arm mode 尚未恢复）：checkAndRestoreAfterAction 处理中，此处不干预
      // restore_time_ 非零时 arm mode 已恢复，应放行让正常 sync 逻辑接管
    }
    else if (skip_sync_after_restore_ <= 0)
    {
      syncArmControllerMode(getCurrentGaitStyleIndex());
    }
    const bool arm_mode_switched_to_1 = (arm_mode_before_sync != 1 && arm_controller_->getMode() == 1);
    if (skip_sync_after_restore_ > 0) skip_sync_after_restore_--;

    double dt = dt_;
    if (dt <= 0.0 || dt > 0.1)
    {
      dt = 0.002;
    }

    CommandDataRL cmdData;
    if (gait_receiver_)
    {
      cmdData = gait_receiver_->getCurrentCommand();
    }

    // mode 2：VR/离线动作接管，调用完整 update 处理轨迹输入
    //   动作期间 Python 轨迹通过 /kuavo_arm_traj → ArmController 正常进入，
    //   arm_controller_->update() 处理后写入 joint_cmd（在 actionToJointCmd 之后，不会被覆盖）
    // mode 1：调 update() 而非 updateInternalState()，让 smoothstep 在切 style 时跑完
    if (arm_controller_->getMode() == 2)
    {
      arm_controller_->update(time, dt, full_joint_pos, full_joint_vel, static_cast<int>(cmdData.cmdStance_), joint_cmd);
    }
    else
    {
      arm_controller_->update(time, dt, full_joint_pos, full_joint_vel, static_cast<int>(cmdData.cmdStance_), joint_cmd);
      // 仅当 style 2→1 切换（arm mode 从 2 切到 1 的首帧）才触发 armRlTakeoverBlend
      // isActive() 检查防止与 standing→walking 触发重复启动
      if (arm_mode_switched_to_1 && static_cast<int>(cmdData.cmdStance_) == 0
          && !arm_takeover_blender_.isActive())
      {
        last_stance_state_for_blend_ = true;
      }
    }

    if (arm_controller_->getMode() == 1)
    {
      return false;
    }
    return true;
  }

  void MoREController::initWaistControl()
  {
    if (waist_command_replacement_enabled_ && waistNum_ > 0)
    {
      try
      {
        waist_controller_ = std::make_unique<WaistController>(nh_, waistNum_, ros_logger_, is_real_);

        Eigen::VectorXd waist_kp, waist_kd;
        if (waist_kp_from_config_.size() == waistNum_ && waist_kd_from_config_.size() == waistNum_)
        {
          waist_kp = waist_kp_from_config_;
          waist_kd = waist_kd_from_config_;
        }
        else
        {
          waist_kp = Eigen::VectorXd::Constant(waistNum_, 10.0);
          waist_kd = Eigen::VectorXd::Constant(waistNum_, 2.0);
          ROS_WARN("[%s] Waist kp/kd not loaded from config, using default values (kp=10.0, kd=2.0)",
                   name_.c_str());
        }

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

        waist_controller_->loadSettings(waist_kp, waist_kd, default_waist_pos, waist_mode_interpolation_velocity_,
                                      waist_mode2_cutoff_freq_);
        waist_controller_->enable(true);
        syncArmControllerMode(style_command_index_);

        ROS_INFO("[%s] Waist controller initialized (waist_joints=%d, mode=%d)", name_.c_str(), waistNum_,
                 waist_controller_->getMode());
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

  bool MoREController::updateWaistCommand(const ros::Time& time,
                                          const SensorData& sensor_data,
                                          kuavo_msgs::jointCmd& joint_cmd)
  {
    if (!waist_command_replacement_enabled_ || waistNum_ == 0 || !waist_controller_)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[%s] updateWaistCommand: disabled or no controller (enabled=%d, waistNum_=%d, controller=%p)",
                        name_.c_str(), waist_command_replacement_enabled_, waistNum_, waist_controller_.get());
      return false;
    }

    // 动作期间不让 auto style 覆盖 callback 设的 style 2
    if (!action_pending_restore_ && skip_sync_after_restore_ <= 0)
    {
      syncArmControllerMode(getCurrentGaitStyleIndex());
    }
    if (skip_sync_after_restore_ > 0) skip_sync_after_restore_--;

    double dt = dt_;
    if (dt <= 0.0 || dt > 0.1)
    {
      dt = 0.002;
    }

    CommandDataRL cmdData;
    if (gait_receiver_)
    {
      cmdData = gait_receiver_->getCurrentCommand();
    }

    Eigen::VectorXd full_joint_pos(jointNum_ + waistNum_ + jointArmNum_);
    Eigen::VectorXd full_joint_vel(jointNum_ + waistNum_ + jointArmNum_);

    if (is_roban_)
    {
      const Eigen::VectorXd waist_pos = sensor_data.jointPos_.segment(0, waistNum_);
      const Eigen::VectorXd leg_pos = sensor_data.jointPos_.segment(waistNum_, jointNum_);
      const Eigen::VectorXd arm_pos = sensor_data.jointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
      const Eigen::VectorXd waist_vel = sensor_data.jointVel_.segment(0, waistNum_);
      const Eigen::VectorXd leg_vel = sensor_data.jointVel_.segment(waistNum_, jointNum_);
      const Eigen::VectorXd arm_vel = sensor_data.jointVel_.segment(waistNum_ + jointNum_, jointArmNum_);
      full_joint_pos << leg_pos, waist_pos, arm_pos;
      full_joint_vel << leg_vel, waist_vel, arm_vel;
    }
    else
    {
      full_joint_pos = sensor_data.jointPos_.head(jointNum_ + waistNum_ + jointArmNum_);
      full_joint_vel = sensor_data.jointVel_.head(jointNum_ + waistNum_ + jointArmNum_);
    }

    waist_controller_->update(time, dt, full_joint_pos, full_joint_vel, static_cast<int>(cmdData.cmdStance_),
                              joint_cmd, jointNum_);

    if (getCurrentGaitStyleIndex() == 0 && waist_controller_->getMode() == 2)
    {
      pose_waist_control_mode_ = 2;
    }

    if (waist_controller_->getMode() == 1)
    {
      return false;
    }
    return true;
  }

  void MoREController::applyUpperBodyDefaultJointCmd(kuavo_msgs::jointCmd& joint_cmd)
  {
    const int total_body = jointNum_ + jointArmNum_ + waistNum_;
    if (static_cast<int>(joint_cmd.joint_q.size()) < total_body)
    {
      return;
    }

    auto setJointDefault = [&](int idx, double default_q) {
      if (idx < 0 || idx >= static_cast<int>(joint_cmd.joint_q.size()))
      {
        return;
      }
      joint_cmd.joint_q[idx] = default_q;
      if (idx < static_cast<int>(joint_cmd.joint_v.size()))
      {
        joint_cmd.joint_v[idx] = 0.0;
      }
      if (idx < static_cast<int>(joint_cmd.tau.size()))
      {
        joint_cmd.tau[idx] = 0.0;
      }
    };

    if (is_roban_)
    {
      for (int i = 0; i < waistNum_; ++i)
      {
        setJointDefault(i, defalutJointPosRL_[i]);
      }
      const int arm_start_rl = waistNum_ + jointNum_;
      const int arm_start_cmd = jointNum_ + waistNum_;
      for (int i = 0; i < jointArmNum_; ++i)
      {
        setJointDefault(arm_start_cmd + i, defalutJointPosRL_[arm_start_rl + i]);
      }
    }
    else
    {
      for (int i = 0; i < waistNum_; ++i)
      {
        setJointDefault(jointNum_ + i, defalutJointPosRL_[jointNum_ + i]);
      }
      for (int i = 0; i < jointArmNum_; ++i)
      {
        setJointDefault(jointNum_ + waistNum_ + i, defalutJointPosRL_[jointNum_ + waistNum_ + i]);
      }
    }
  }

  void MoREController::changeRuiwoMotorParamAsync(const std::string& param_name)
  {
    if (!is_real_ || !use_amp_ruiwo_kpkd_)
    {
      return;
    }
    if (!srv_change_motor_param_)
    {
      srv_change_motor_param_ =
          nh_.serviceClient<kuavo_msgs::ExecuteArmAction>("/hardware/change_ruiwo_motor_param");
    }

    std::thread([this, param_name]() {
      try
      {
        if (!srv_change_motor_param_.waitForExistence(ros::Duration(2.0)))
        {
          ROS_WARN("[%s] Ruiwo motor param service not available", name_.c_str());
          return;
        }
        kuavo_msgs::ExecuteArmAction srv;
        srv.request.action_name = param_name;
        if (srv_change_motor_param_.call(srv) && srv.response.success)
        {
          ROS_INFO("[%s] Ruiwo motor param switched to %s", name_.c_str(), param_name.c_str());
        }
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("[%s] changeRuiwoMotorParamAsync failed: %s", name_.c_str(), e.what());
      }
    }).detach();
  }

  void MoREController::updateVelocityLimitsParam(ros::NodeHandle& nh)
  {
    std::vector<double> limits_vec(6);
    limits_vec[0] = velocityLimits_(0);
    limits_vec[1] = velocityLimits_(1);
    limits_vec[2] = velocityLimits_(2);
    limits_vec[3] = 0.0;
    limits_vec[4] = 0.0;
    limits_vec[5] = velocityLimits_(3);

    nh.setParam("/velocity_limits", limits_vec);
    nh.setParam("/velocity_limit_ratio", 1.0);
    nh.setParam("/cmd_vel_line_x_neg_scale", cmdVelLineXNegScale_);

    ROS_INFO("[%s] Updated /velocity_limits: vx=[%.2f, %.2f] m/s, wz=±%.2f rad/s",
             name_.c_str(), -limits_vec[0] * cmdVelLineXNegScale_, limits_vec[0], limits_vec[5]);
    ROS_INFO("[%s] /velocity_limits rosparam: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             name_.c_str(), limits_vec[0], limits_vec[1], limits_vec[2], limits_vec[3], limits_vec[4],
             limits_vec[5]);
  }

}  // namespace humanoid_controller
