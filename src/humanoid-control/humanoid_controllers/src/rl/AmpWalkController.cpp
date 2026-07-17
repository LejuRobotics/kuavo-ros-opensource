// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/AmpWalkController.h"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <angles/angles.h>
#include "kuavo_common/common/common.h"
#include <ros/package.h>
#include <std_msgs/Int32.h>
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
    is_amp_hand_controller_ = (name == "amp_hand_controller");
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
    gait_receiver_->setAmpHandController(name_ == "amp_hand_controller");
    
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
    std::string ankle_solver_type = "4gen_pro"; // 默认值
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

    // AMP 模式切换服务：每个 AMP 控制器独立命名，避免多实例抢占同一服务
    const std::string change_amp_mode_srv_name =
        "/humanoid_controller/" + name_ + "/change_amp_mode";
    const std::string amp_mode_event_topic =
        "/humanoid_controller/" + name_ + "/amp_mode";
    amp_mode_event_pub_ = nh_.advertise<std_msgs::Int32>(amp_mode_event_topic, 1, true);
    change_amp_mode_srv_ = nh_.advertiseService(change_amp_mode_srv_name,
                                                &AmpWalkController::changeAmpModeCallback,
                                                this);
    publishAmpModeEvent();

    initialized_ = true;

    ROS_INFO("[%s] AmpWalkController initialized (change_amp_mode: %s, amp_mode_event: %s)",
             name_.c_str(), change_amp_mode_srv_name.c_str(), amp_mode_event_topic.c_str());
    return true;
  }

  bool AmpWalkController::changeAmpModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                                                kuavo_msgs::changeArmCtrlMode::Response &res)
  {
    int requested_mode = req.control_mode;

    if (requested_mode < 0 || requested_mode > 2)
    {
      ROS_WARN("[%s] Received invalid AMP mode %d, valid range is [0, 2]. Keep current mode %d.",
               name_.c_str(), requested_mode, amp_mode_);
      res.result = false;
      res.mode = amp_mode_;
      res.message = "Invalid AMP mode, must be 0, 1 or 2";
      return true;
    }

    if (amp_mode_ == 1 && requested_mode == 0 && isSquatPostureDefenseActive())
    {
      const SensorData sensor_data = getRobotSensorData();
      const double knee_l = sensor_data.jointPos_[kStandUpLegL4ActionIdx_];
      const double knee_r = sensor_data.jointPos_[kStandUpLegR4ActionIdx_];
      const geometry_msgs::Twist smoothed_vel = gait_receiver_->getSmoothedCmdVel();
      const double processed_cmdangz =
          applyTinyCmdAngzClip(smoothed_vel.angular.z);
      ROS_WARN_THROTTLE(0.5,
                        "[%s] Squat posture defense: reject exit from posture mode "
                        "(knee_l=%.3f, knee_r=%.3f, kneeThreshold=%.3f, processed_cmdangz=%.3f, "
                        "absCmdAngzLimit=%.3f).",
                        name_.c_str(), knee_l, knee_r,
                        squat_posture_defense_height_threshold_, processed_cmdangz,
                        squat_posture_defense_abs_cmdangz_limit_);
      res.result = false;
      res.mode = amp_mode_;
      res.message = "Squat posture defense active, cannot exit posture control mode";
      return true;
    }

    amp_mode_ = requested_mode;
    squat_posture_deep_seen_ = false;
    squat_auto_stand_up_frames_remaining_ = 0;
    squat_auto_exit_yaw_frames_remaining_ = 0;
    publishAmpModeEvent();

    ROS_INFO("[%s] AMP mode switched to %d (0: pure AMP walk, 1: stance/bend/squat with arms, 2: walk with arms).",
             name_.c_str(), amp_mode_);

    res.result = true;
    res.mode = amp_mode_;
    res.message = "AMP mode updated successfully";
    return true;
  }

  void AmpWalkController::publishAmpModeEvent()
  {
    if (!amp_mode_event_pub_)
    {
      return;
    }
    std_msgs::Int32 msg;
    msg.data = amp_mode_;
    amp_mode_event_pub_.publish(msg);
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

    standDefaultJointPosRL_.resize(defalutJointPosRL_.size());
    standDefaultJointPosRL_ = defalutJointPosRL_;
    use_stand_default_joint_state_ = false;
    has_stand_default_joint_state_ = false;
    stand_velocity_threshold_ = 0.1;
    stand_angular_velocity_threshold_ = 0.1;
    loadData::loadPtreeValue(pt, use_stand_default_joint_state_, "use_stand_default_joint_state", false);

    if (use_stand_default_joint_state_ && pt.get_child_optional("standdefaultJointState"))
    {
      loadEigenMatrix("standdefaultJointState", standDefaultJointPosRL_);
      if (standDefaultJointPosRL_.size() == defalutJointPosRL_.size())
      {
        has_stand_default_joint_state_ = true;
        loadData::loadPtreeValue(pt, stand_velocity_threshold_, "standVelocityThreshold", false);
        loadData::loadPtreeValue(pt, stand_angular_velocity_threshold_,
                                 "standAngularVelocityThreshold", false);
      }
      else
      {
        standDefaultJointPosRL_ = defalutJointPosRL_;
      }
    }

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

    if (is_amp_hand_controller_)
    {
      cmdVelLineXLow_ = velocityLimits_(0);
      cmdVelLineXUp_ = velocityLimits_(0);
      loadData::loadPtreeValue(pt, cmdVelLineXLow_, "cmdVelLineXlow", false);
      loadData::loadPtreeValue(pt, cmdVelLineXUp_, "cmdVelLineXup", false);
      loadData::loadPtreeValue(pt, cmdVelLineXNeg_, "cmdVelLineXNeg", false);
      loadData::loadPtreeValue(pt, squatHeightMin_, "squatHeightMin", false);
      loadData::loadPtreeValue(pt, squatHeightMax_, "squatHeightMax", false);
      if (pt.find("squatPostureDefense") != pt.not_found())
      {
        loadData::loadPtreeValue(pt, squat_posture_defense_enabled_,
                                 "squatPostureDefense.enabled", false);
        loadData::loadPtreeValue(pt, squat_posture_defense_height_threshold_,
                                 "squatPostureDefense.heightThreshold", false);
        loadData::loadPtreeValue(pt, squat_posture_defense_abs_cmdangz_limit_,
                                 "squatPostureDefense.absCmdAngzLimit", false);
        ROS_INFO("[%s] SquatPostureDefense: enabled=%s, heightThreshold=%.3f, absCmdAngzLimit=%.3f",
                 name_.c_str(),
                 squat_posture_defense_enabled_ ? "true" : "false",
                 squat_posture_defense_height_threshold_,
                 squat_posture_defense_abs_cmdangz_limit_);
      }
      loadData::loadPtreeValue(pt, ampVRcmdvelLinearXLimit_, "ampVRcmdvelLinearXLimit", false);
      loadData::loadPtreeValue(pt, ampVRcmdvelLinearYLimit_, "ampVRcmdvelLinearYLimit", false);
      loadData::loadPtreeValue(pt, ampVRcmdvelLinearZLimit_, "ampVRcmdvelLinearZLimit", false);
      loadData::loadPtreeValue(pt, ampVRcmdvelAngularYAWLimit_, "ampVRcmdvelAngularYAWLimit", false);
      loadData::loadPtreeValue(pt, use_virtual_arm_obs_, "use_virtual_arm_obs", false);
      loadData::loadPtreeValue(pt, lateral_elbow_fix_, "lateral_elbow_fix", false);
      loadData::loadPtreeValue(pt, enable_elbow_scale_, "enable_elbow_scale", false);
      loadData::loadPtreeValue(pt, enable_back_arm_enhance_, "enable_back_arm_enhance", false);
      loadData::loadPtreeValue(pt, enable_standup_enhance_, "enable_standup_enhance", false);
      loadData::loadPtreeValue(pt, enable_roll_compensation_closed_loop_,
                               "enable_roll_compensation_closed_loop", false);
      if (pt.find("rollCompensationClosedLoop") != pt.not_found())
      {
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_cmd_x_min_,
                                 "rollCompensationClosedLoop.cmdXMin", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_cmd_x_max_,
                                 "rollCompensationClosedLoop.cmdXMax", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_abs_cmd_y_max_,
                                 "rollCompensationClosedLoop.absCmdYMax", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_abs_cmd_ang_z_max_,
                                 "rollCompensationClosedLoop.absCmdAngZMax", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_filter_time_constant_sec_,
                                 "rollCompensationClosedLoop.filterTimeConstantSec", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_target_time_constant_sec_,
                                 "rollCompensationClosedLoop.targetTimeConstantSec", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_kp_,
                                 "rollCompensationClosedLoop.kp", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_ki_,
                                 "rollCompensationClosedLoop.ki", false);
        loadData::loadPtreeValue(pt, roll_compensation_closed_loop_max_deg_,
                                 "rollCompensationClosedLoop.maxCompensationDeg", false);
      }
      roll_compensation_closed_loop_cmd_x_min_ =
          std::max(roll_compensation_closed_loop_cmd_x_min_, 0.0);
      roll_compensation_closed_loop_cmd_x_max_ =
          std::max(roll_compensation_closed_loop_cmd_x_max_,
                   roll_compensation_closed_loop_cmd_x_min_ + 1e-3);
      roll_compensation_closed_loop_filter_time_constant_sec_ =
          std::max(roll_compensation_closed_loop_filter_time_constant_sec_, 1e-3);
      roll_compensation_closed_loop_target_time_constant_sec_ =
          std::max(roll_compensation_closed_loop_target_time_constant_sec_, 1e-3);
      roll_compensation_closed_loop_ki_ =
          std::max(roll_compensation_closed_loop_ki_, 0.0);
      roll_compensation_closed_loop_max_deg_ =
          std::max(roll_compensation_closed_loop_max_deg_, 0.0);
      loadData::loadPtreeValue(pt, enable_off_cmdy_by_cmdx_, "enable_off_cmdy_by_cmdx", false);
      loadData::loadPtreeValue(pt, enable_off_cmdy_by_cmdangz_, "enable_off_cmdy_by_cmdangz", false);
      try
      {
        Eigen::Matrix<double, 6, 1> tiny_cmd_clip;
        loadEigenMatrix("TinyCmdClip", tiny_cmd_clip);
        tiny_cmdx_clip_pos_min_ = tiny_cmd_clip(0);
        tiny_cmdx_clip_pos_max_ = tiny_cmd_clip(1);
        tiny_cmdy_clip_min_ = tiny_cmd_clip(2);
        tiny_cmdy_clip_max_ = tiny_cmd_clip(3);
        tiny_cmd_angz_clip_min_ = tiny_cmd_clip(4);
        tiny_cmd_angz_clip_max_ = tiny_cmd_clip(5);
        tiny_cmdx_clip_enabled_ = tiny_cmdx_clip_pos_max_ > tiny_cmdx_clip_pos_min_;
        tiny_cmdy_clip_enabled_ = tiny_cmdy_clip_max_ > tiny_cmdy_clip_min_;
        tiny_cmd_angz_clip_enabled_ = tiny_cmd_angz_clip_max_ > tiny_cmd_angz_clip_min_;
        ROS_INFO("[%s] TinyCmdClip: pos[%.3f, %.3f)->%.3f, "
                 "cmd_y[%.3f, %.3f)->%.3f, angz[%.3f, %.3f)->%.3f, enabled=%s/%s/%s",
                 name_.c_str(),
                 tiny_cmdx_clip_pos_min_, tiny_cmdx_clip_pos_max_, tiny_cmdx_clip_pos_max_,
                 tiny_cmdy_clip_min_, tiny_cmdy_clip_max_, tiny_cmdy_clip_max_,
                 tiny_cmd_angz_clip_min_, tiny_cmd_angz_clip_max_, tiny_cmd_angz_clip_max_,
                 tiny_cmdx_clip_enabled_ ? "true" : "false",
                 tiny_cmdy_clip_enabled_ ? "true" : "false",
                 tiny_cmd_angz_clip_enabled_ ? "true" : "false");
      }
      catch (const std::exception& e)
      {
        tiny_cmdx_clip_enabled_ = false;
        tiny_cmdy_clip_enabled_ = false;
        tiny_cmd_angz_clip_enabled_ = false;
        ROS_WARN("[%s] TinyCmdClip not loaded: %s", name_.c_str(), e.what());
      }

      if (pt.find("lowSpeedKickStart") != pt.not_found())
      {
        loadData::loadPtreeValue(pt, low_speed_kick_enabled_, "lowSpeedKickStart.enabled", false);
        loadData::loadPtreeValue(pt, low_speed_kick_velocity_, "lowSpeedKickStart.kickVelocity", false);
        loadData::loadPtreeValue(pt, low_speed_kick_duration_steps_, "lowSpeedKickStart.durationSteps", false);
        loadData::loadPtreeValue(pt, low_speed_kick_rest_threshold_, "lowSpeedKickStart.restCmdThreshold", false);
        loadData::loadPtreeValue(pt, low_speed_kick_trigger_velocity_, "lowSpeedKickStart.triggerVelocity", false);
        loadData::loadPtreeValue(pt, low_speed_kick_trigger_tolerance_, "lowSpeedKickStart.triggerTolerance", false);
        loadData::loadPtreeValue(pt, low_speed_kick_lateral_threshold_, "lowSpeedKickStart.lateralThreshold", false);
        loadData::loadPtreeValue(pt, low_speed_kick_yaw_threshold_, "lowSpeedKickStart.yawThreshold", false);
        low_speed_kick_duration_steps_ = std::max(low_speed_kick_duration_steps_, 1);
        low_speed_kick_trigger_tolerance_ = std::max(low_speed_kick_trigger_tolerance_, 0.0);
        ROS_INFO("[%s] LowSpeedKickStart: enabled=%s, kickVel=%.2f, steps=%d, "
                 "trigger=%.2f±%.2f, restTh=%.3f",
                 name_.c_str(),
                 low_speed_kick_enabled_ ? "true" : "false",
                 low_speed_kick_velocity_,
                 low_speed_kick_duration_steps_,
                 low_speed_kick_trigger_velocity_,
                 low_speed_kick_trigger_tolerance_,
                 low_speed_kick_rest_threshold_);
      }

      if (pt.find("velocitySmoothing") != pt.not_found())
      {
        loadData::loadPtreeValue(pt, stance_height_stand_up_smoothing_enabled_,
                                 "velocitySmoothing.standUpHeightSmoothingEnabled", false);
        loadData::loadPtreeValue(pt, max_stance_height_stand_up_change_,
                                 "velocitySmoothing.maxStandUpHeightChange", false);
        loadData::loadPtreeValue(pt, stance_height_smooth_start_,
                                 "velocitySmoothing.StandUpHeightSmoothStart", false);
        ROS_INFO("[%s] Stance height stand-up smoothing: enabled=%s, maxChange=%.4f m/step, "
                 "smoothStart=%.4f m",
                 name_.c_str(),
                 stance_height_stand_up_smoothing_enabled_ ? "true" : "false",
                 max_stance_height_stand_up_change_,
                 stance_height_smooth_start_);
      }
    }

    // 加载手臂控制参数（用于 ArmController）
    if (arm_command_replacement_enabled && jointArmNum_ > 0)
    {
      loadData::loadPtreeValue(pt, arm_max_tracking_velocity_, "armVelocityLimit.maxTrackingVelocity", false);
      loadData::loadPtreeValue(pt, arm_tracking_error_threshold_, "armVelocityLimit.trackingErrorThreshold", false);
      loadData::loadPtreeValue(pt, arm_mode_interpolation_velocity_, "armVelocityLimit.modeInterpolationVelocity", false);
      loadData::loadPtreeValue(pt, arm_rl_takeover_blend_enabled_, "armRlTakeoverBlend.enabled", false);
      loadData::loadPtreeValue(pt, arm_rl_takeover_blend_duration_, "armRlTakeoverBlend.duration", false);
      loadData::loadPtreeValue(pt, arm_zero_action_in_standing_, "armRlTakeoverBlend.zeroActionInStanding", false);
      arm_takeover_blender_.configure(arm_rl_takeover_blend_enabled_, arm_rl_takeover_blend_duration_);
      
      ROS_INFO("[%s] Arm control parameters loaded: max_velocity=%.3f rad/s, error_threshold=%.3f rad, mode_interpolation_velocity=%.3f rad/s",
               name_.c_str(), arm_max_tracking_velocity_, arm_tracking_error_threshold_, arm_mode_interpolation_velocity_);
      ROS_INFO("[%s] Arm RL takeover blend: enabled=%s, duration=%.3f s, zero_action_in_standing=%s",
               name_.c_str(),
               arm_rl_takeover_blend_enabled_ ? "true" : "false",
               arm_rl_takeover_blend_duration_,
               arm_zero_action_in_standing_ ? "true" : "false");
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

    // 加载 X 负向单独缩放系数（非 amp_hand_controller 使用）
    if (!is_amp_hand_controller_)
    {
      loadData::loadPtreeValue(pt, cmdVelLineXNegScale_, "commandData.scale.cmdVelLineXNegScale", false);
      loadData::loadPtreeValue(pt, cmdVelLineXNegScaleExternalArm_, "commandData.scale.cmdVelLineXNegScaleExternalArm", false);
    }

    ROS_INFO("[%s] loadConfig done. num_actions_=%d, numSingleObs_=%d, frameStack_=%d",
             name_.c_str(), num_actions_, numSingleObs_, frameStack_);
    return true;
  }

  const Eigen::VectorXd& AmpWalkController::getActiveDefaultJointPos(const CommandDataRL& cmd) const
  {
    const bool is_truly_standing =
        has_stand_default_joint_state_ &&
        cmd.cmdStance_ >= 0.5 &&
        std::abs(cmd.cmdVelLineX_) < stand_velocity_threshold_ &&
        std::abs(cmd.cmdVelAngularZ_) < stand_angular_velocity_threshold_;
    return is_truly_standing ? standDefaultJointPosRL_ : defalutJointPosRL_;
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
    arm_takeover_blender_.reset();
    last_stance_state_for_blend_ = true;  // 初始化为站立状态
    smoothed_stance_height_cmd_ = 0.0;
    squat_posture_deep_seen_ = false;
    squat_auto_stand_up_frames_remaining_ = 0;
    squat_auto_exit_yaw_frames_remaining_ = 0;
    roll_compensation_closed_loop_initialized_ = false;
    roll_compensation_filtered_roll_rad_ = 0.0;
    roll_compensation_target_roll_rad_ = 0.0;
    roll_compensation_integral_rad_sec_ = 0.0;
    
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
      gait_receiver_->resetVelocityState();
      gait_receiver_->setEnabled(true);
    }

    // Ruiwo 手臂增益已通过 motor_pdo_kp/kd（skw_rl_param.info）在 joint_cmd 中实时下发

    ROS_INFO("[%s] Controller resumed, reset state", name_.c_str());
    reset();
  }

  bool AmpWalkController::requestToExit() const
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

  bool AmpWalkController::isAllowToExit() const
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

  double AmpWalkController::applyTinyCmdxClip(double cmdx) const
  {
    if (!tiny_cmdx_clip_enabled_)
    {
      return cmdx;
    }
    if (cmdx >= tiny_cmdx_clip_pos_min_ && cmdx < tiny_cmdx_clip_pos_max_)
    {
      return tiny_cmdx_clip_pos_max_;
    }
    return cmdx;
  }

  double AmpWalkController::applyTinyCmdYClip(double cmdy) const
  {
    if (!tiny_cmdy_clip_enabled_)
    {
      return cmdy;
    }
    const double abs_cmdy = std::abs(cmdy);
    if (abs_cmdy >= tiny_cmdy_clip_min_ && abs_cmdy < tiny_cmdy_clip_max_)
    {
      return cmdy >= 0.0 ? tiny_cmdy_clip_max_ : -tiny_cmdy_clip_max_;
    }
    return cmdy;
  }

  double AmpWalkController::applyTinyCmdAngzClip(double angz) const
  {
    if (!tiny_cmd_angz_clip_enabled_)
    {
      return angz;
    }
    const double abs_angz = std::abs(angz);
    if (abs_angz >= tiny_cmd_angz_clip_min_ && abs_angz < tiny_cmd_angz_clip_max_)
    {
      return angz >= 0.0 ? tiny_cmd_angz_clip_max_ : -tiny_cmd_angz_clip_max_;
    }
    return angz;
  }

  void AmpWalkController::applyLowSpeedKickStart(CommandDataRL& cmd)
  {
    if (!low_speed_kick_enabled_ || !is_amp_hand_controller_)
    {
      return;
    }

    const auto cancel_kick = [&]() { low_speed_kick_remaining_steps_ = 0; };

    if (cmd.cmdStance_ >= 1.0)
    {
      cancel_kick();
      return;
    }

    const bool in_kick_trigger_range =
        std::abs(cmd.cmdVelLineX_ - low_speed_kick_trigger_velocity_) <=
            low_speed_kick_trigger_tolerance_ &&
        std::abs(cmd.cmdVelLineY_) < low_speed_kick_lateral_threshold_ &&
        std::abs(cmd.cmdVelAngularZ_) < low_speed_kick_yaw_threshold_;

    if (!in_kick_trigger_range)
    {
      cancel_kick();
      return;
    }

    if (low_speed_kick_remaining_steps_ > 0)
    {
      cmd.cmdVelLineX_ = low_speed_kick_velocity_;
      --low_speed_kick_remaining_steps_;
      return;
    }

    if (std::abs(prev_processed_cmd_vel_line_x_) < low_speed_kick_rest_threshold_)
    {
      cmd.cmdVelLineX_ = low_speed_kick_velocity_;
      low_speed_kick_remaining_steps_ = low_speed_kick_duration_steps_ - 1;
      ROS_INFO_THROTTLE(1.0, "[%s] Low-speed kick start: cmd_x %.2f for %d steps",
                        name_.c_str(), low_speed_kick_velocity_, low_speed_kick_duration_steps_);
    }
  }

  void AmpWalkController::applyStanceHeightStandUpSmoothing(CommandDataRL& cmd)
  {
    stand_up_rising_active_ = false;

    if (!is_amp_hand_controller_)
    {
      return;
    }

    if (cmd.cmdStance_ < 1.0)
    {
      smoothed_stance_height_cmd_ = 0.0;
      return;
    }

    const bool in_posture_stance = (amp_mode_ == 1);
    const double raw_height = in_posture_stance ? cmd.cmdVelAngularZ_ : 0.0;
    const double diff = raw_height - smoothed_stance_height_cmd_;

    stand_up_rising_active_ =
        (in_posture_stance || std::abs(smoothed_stance_height_cmd_) > 1e-6) &&
        diff > kStandUpHeightRisingEpsilon_;

    if (!stance_height_stand_up_smoothing_enabled_)
    {
      return;
    }

    if (diff > kStandUpHeightRisingEpsilon_)
    {
      if (smoothed_stance_height_cmd_ < stance_height_smooth_start_)
      {
        // 到达阈值前按单步变化量平滑，且本周期不跨过阈值。
        smoothed_stance_height_cmd_ += std::min(diff, max_stance_height_stand_up_change_);
        smoothed_stance_height_cmd_ =
            std::min(smoothed_stance_height_cmd_, stance_height_smooth_start_);
      }
      else
      {
        // 达到阈值后直接跟随剩余起身高度。
        smoothed_stance_height_cmd_ = raw_height;
      }
    }
    else
    {
      smoothed_stance_height_cmd_ = raw_height;
    }

    smoothed_stance_height_cmd_ = std::max(smoothed_stance_height_cmd_, -max_stance_squat_depth_);

    if (in_posture_stance || std::abs(smoothed_stance_height_cmd_) > 1e-6)
    {
      cmd.cmdVelAngularZ_ = smoothed_stance_height_cmd_;
      cmd.cmdVelLineZ_ = smoothed_stance_height_cmd_;
    }

  }

  void AmpWalkController::applySquatPostureAutoExit(CommandDataRL& cmd,
                                                    const SensorData& sensor_data)
  {
    if (!is_amp_hand_controller_ || !squat_posture_defense_enabled_ ||
        sensor_data.jointPos_.size() <= kStandUpLegR4ActionIdx_)
    {
      return;
    }

    const double knee_l = sensor_data.jointPos_[kStandUpLegL4ActionIdx_];
    const double knee_r = sensor_data.jointPos_[kStandUpLegR4ActionIdx_];

    if (amp_mode_ == 1)
    {
      if (knee_l > squat_posture_defense_height_threshold_ &&
          knee_r > squat_posture_defense_height_threshold_)
      {
        squat_posture_deep_seen_ = true;
      }

      // 高度命令通常先于实际关节到位；deep_seen_ 已锁存“曾处于深蹲”，
      // 因此双膝随后回落到阈值以下即可可靠判定物理起身完成。
      if (squat_posture_deep_seen_ &&
          knee_l < squat_posture_defense_height_threshold_ &&
          knee_r < squat_posture_defense_height_threshold_)
      {
        squat_posture_deep_seen_ = false;
        squat_auto_stand_up_frames_remaining_ = kSquatAutoStandUpFrames_;
        ROS_WARN("[%s] Squat posture auto stand-up: knee_l=%.3f, knee_r=%.3f, "
                 "send height=%.3f for %d frames.",
                 name_.c_str(), knee_l, knee_r, squatHeightMax_,
                 kSquatAutoStandUpFrames_);
      }
    }

    if (squat_auto_stand_up_frames_remaining_ > 0)
    {
      cmd.setzero();
      cmd.cmdVelLineZ_ = squatHeightMax_;
      cmd.cmdVelAngularZ_ = squatHeightMax_;
      --squat_auto_stand_up_frames_remaining_;
      if (squat_auto_stand_up_frames_remaining_ == 0)
      {
        amp_mode_ = 0;
        smoothed_stance_height_cmd_ = 0.0;
        squat_auto_exit_yaw_frames_remaining_ = kSquatAutoExitYawFrames_;
        publishAmpModeEvent();
        ROS_WARN("[%s] Squat posture auto stand-up complete, exit posture mode "
                 "and send cmd_angz=%.2f for %d frames.",
                 name_.c_str(), kSquatAutoExitYawCmd_,
                 kSquatAutoExitYawFrames_);
      }
    }
    else if (squat_auto_exit_yaw_frames_remaining_ > 0)
    {
      cmd.setzero();
      cmd.cmdStance_ = 0.0;
      cmd.cmdVelAngularZ_ = kSquatAutoExitYawCmd_;
      --squat_auto_exit_yaw_frames_remaining_;
    }
  }

  bool AmpWalkController::isSquatPostureDefenseActive() const
  {
    if (!is_amp_hand_controller_ || !squat_posture_defense_enabled_ || !gait_receiver_ ||
        amp_mode_ != 1)
    {
      return false;
    }

    const SensorData sensor_data = getRobotSensorData();
    const double knee_l = sensor_data.jointPos_[kStandUpLegL4ActionIdx_];
    const double knee_r = sensor_data.jointPos_[kStandUpLegR4ActionIdx_];
    const geometry_msgs::Twist smoothed_vel = gait_receiver_->getSmoothedCmdVel();
    const double processed_cmdangz = applyTinyCmdAngzClip(smoothed_vel.angular.z);

    return knee_l > squat_posture_defense_height_threshold_ &&
           knee_r > squat_posture_defense_height_threshold_ &&
           std::abs(processed_cmdangz) < squat_posture_defense_abs_cmdangz_limit_;
  }

  void AmpWalkController::updatePhase(const CommandDataRL& cmd)
  {
    // 基本照 humanoidController_rl.cpp::updatePhase
    // 根据速度方向选择正负限制：速度>0 用正向限制，速度<0 用负向限制
    double velLimitX = (cmd.cmdVelLineX_ >= 0)
                           ? velocityLimits_(0)
                           : (is_amp_hand_controller_ ? cmdVelLineXNeg_ : velocityLimits_(1));
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

    applyStanceHeightStandUpSmoothing(cmd);
    applySquatPostureAutoExit(cmd, sensor_data);
    
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

    Eigen::VectorXd command_state(1);
    command_state << cmd.cmdStance_;
    // 速度命令 [vx, vy, omega_z]
    cmd.scale();
    const bool external_arm_control_active = is_amp_hand_controller_ &&
                                             arm_command_replacement_enabled_ &&
                                             jointArmNum_ > 0 && arm_controller_ &&
                                             arm_controller_->getMode() != 1;

    // amp_hand: 负向 cmd_x 限速；其他控制器仍使用负向缩放系数
    if (cmd.cmdVelLineX_ < 0.0)
    {
      if (is_amp_hand_controller_)
      {
        cmd.cmdVelLineX_ = std::max(cmd.cmdVelLineX_, -cmdVelLineXNeg_);
      }
      else
      {
        const double neg_scale = (arm_command_replacement_enabled_ &&
                                  jointArmNum_ > 0 && arm_controller_ &&
                                  arm_controller_->getMode() != 1)
                                     ? cmdVelLineXNegScaleExternalArm_
                                     : cmdVelLineXNegScale_;
        cmd.cmdVelLineX_ *= neg_scale;
      }
    }

    if (is_amp_hand_controller_ && enable_off_cmdy_by_cmdx_)
    {
      if (cmd.cmdVelLineX_ > kOffCmdyByCmdXThreshold_)
      {
        cmd.cmdVelLineY_ = 0.0;
      }
      if (std::abs(cmd.cmdVelLineY_) > kOffCmdxByCmdYThreshold_)
      {
        cmd.cmdVelLineX_ = 0.0;
      }
    }

    if (is_amp_hand_controller_ && enable_off_cmdy_by_cmdangz_)
    {
      if (std::abs(cmd.cmdVelAngularZ_) > kOffCmdyByCmdAngZThreshold_)
      {
        cmd.cmdVelLineY_ = 0.0;
      }
      if (std::abs(cmd.cmdVelLineY_) > kOffCmdAngZByCmdYThreshold_)
      {
        cmd.cmdVelAngularZ_ = 0.0;
      }
    }

    if (is_amp_hand_controller_ &&
        (tiny_cmdx_clip_enabled_ || tiny_cmdy_clip_enabled_ || tiny_cmd_angz_clip_enabled_))
    {
      if (tiny_cmdx_clip_enabled_)
      {
        cmd.cmdVelLineX_ = applyTinyCmdxClip(cmd.cmdVelLineX_);
      }
      if (tiny_cmdy_clip_enabled_)
      {
        cmd.cmdVelLineY_ = applyTinyCmdYClip(cmd.cmdVelLineY_);
      }
      if (tiny_cmd_angz_clip_enabled_ && cmd.cmdStance_ != 1.0 &&
          std::abs(cmd.cmdVelLineX_) < 0.3 && std::abs(cmd.cmdVelLineY_) < 0.2 &&
          std::abs(cmd.cmdVelAngularZ_) > 0.1)
      {
        cmd.cmdVelAngularZ_ = applyTinyCmdAngzClip(cmd.cmdVelAngularZ_);
      }
    }

    applyLowSpeedKickStart(cmd);

    // enable_standup_enhance：下蹲时按深度自动叠加正向 cmdVelLineX_（弯腰通道）
    if (enable_standup_enhance_ && is_amp_hand_controller_ && cmd.cmdStance_ >= 1.0 &&
        !stand_up_rising_active_)
    {
      const double height_cmd = cmd.cmdVelAngularZ_;
      if (height_cmd < kSquatPitchFadeHeightStart_)
      {
        const double pitch_weight = std::clamp(
            -height_cmd / max_stance_squat_depth_,
            0.0, 1.0);
        const double auto_bend = kSquatPitchMaxCmd_ * pitch_weight;
        cmd.cmdVelLineX_ = std::max(cmd.cmdVelLineX_, auto_bend);
      }
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

    prev_processed_cmd_vel_line_x_ = cmd.cmdVelLineX_;
    
    Eigen::VectorXd tempCommand_ = cmd.getCommandRL();
    Eigen::VectorXd tempCommand_scalar_state = tempCommand_;
    tempCommand_scalar_state[3] = 1.0 - tempCommand_scalar_state[3];  // 前3维不变，第4维做 1- 操作    

    // === 2. 状态、IMU、关节等数据，与 humanoidController_rl.cpp 一致 ===
    const Eigen::Vector3d baseEuler(state_est(2), state_est(1), state_est(0));
    const Eigen::Vector3d baseAngVel(state_est(6 + waistNum_ + jointNum_ + jointArmNum_),
                                     state_est(6 + waistNum_ + jointNum_ + jointArmNum_ + 1),
                                     state_est(6 + waistNum_ + jointNum_ + jointArmNum_ + 2));
    const Eigen::Vector3d baseLineVel = state_est.segment(9 + waistNum_ + jointNum_ + jointArmNum_, 3);
    const Eigen::Vector3d basePos = state_est.segment(3, 3);

    const Eigen::VectorXd& active_default = getActiveDefaultJointPos(cmd);
    Eigen::VectorXd jointPos = sensor_data.jointPos_ - active_default;
    Eigen::VectorXd jointVel = sensor_data.jointVel_;

    const bool virtual_arm_obs_active = is_amp_hand_controller_ &&
                                        use_virtual_arm_obs_ &&
                                        external_arm_control_active;
    if (virtual_arm_obs_active)
    {
      const int arm_start_idx = jointNum_ + waistNum_;
      jointPos.segment(arm_start_idx, jointArmNum_).setZero();
      jointVel.segment(arm_start_idx, jointArmNum_).setZero();
    }

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
    Eigen::Vector3d projected_gravity = R.transpose() * gravity_world;
    if (virtual_arm_obs_active)
    {
      const double virtual_arm_obs_pitch_scale =
          cmd.cmdVelLineX_ >= -0.12 ? cmd.cmdVelLineX_ : -0.1;
      const double compensation_pitch_deg =
          virtual_arm_obs_pitch_scale >= -0.005
              ? kVirtualArmObsPitchBaseDeg_ +
                    kVirtualArmObsPitchCompensationDeg_ * virtual_arm_obs_pitch_scale
              : kVirtualArmObsPitchBaseDegNeg_ +
                    kVirtualArmObsPitchCompensationDegNeg_ * virtual_arm_obs_pitch_scale;
      const double compensation_pitch_rad = compensation_pitch_deg * M_PI / 180.0;
      projected_gravity = Eigen::AngleAxisd(-compensation_pitch_rad, Eigen::Vector3d::UnitY()) * projected_gravity;
    }

    const bool is_walking_mode = cmd.cmdStance_ < 0.5;
    double total_roll_compensation_deg = 0.0;
    if (is_amp_hand_controller_ && enable_roll_compensation_closed_loop_)
    {
      const Eigen::Matrix3d sensor_rotation = sensor_data.quat_.toRotationMatrix();
      const double measured_roll_rad = std::atan2(sensor_rotation(2, 1), sensor_rotation(2, 2));
      const double observation_dt =
          inference_frequency_ > 0.0 ? 1.0 / inference_frequency_ : 0.02;

      if (!roll_compensation_closed_loop_initialized_)
      {
        roll_compensation_filtered_roll_rad_ = measured_roll_rad;
        roll_compensation_target_roll_rad_ = measured_roll_rad;
        roll_compensation_integral_rad_sec_ = 0.0;
        roll_compensation_closed_loop_initialized_ = true;
      }

      const double filter_alpha =
          std::exp(-observation_dt / roll_compensation_closed_loop_filter_time_constant_sec_);
      roll_compensation_filtered_roll_rad_ =
          filter_alpha * roll_compensation_filtered_roll_rad_ +
          (1.0 - filter_alpha) * measured_roll_rad;

      if (!is_walking_mode)
      {
        // 站立时缓慢学习本机 IMU/装配的中立 roll，不把静态安装误差带入行走补偿。
        const double target_alpha =
            std::exp(-observation_dt / roll_compensation_closed_loop_target_time_constant_sec_);
        roll_compensation_target_roll_rad_ =
            target_alpha * roll_compensation_target_roll_rad_ +
            (1.0 - target_alpha) * roll_compensation_filtered_roll_rad_;
        roll_compensation_integral_rad_sec_ = 0.0;
      }

      const bool closed_loop_active =
          is_walking_mode &&
          cmd.cmdVelLineX_ > roll_compensation_closed_loop_cmd_x_min_ &&
          cmd.cmdVelLineX_ < roll_compensation_closed_loop_cmd_x_max_ &&
          std::abs(cmd.cmdVelLineY_) < roll_compensation_closed_loop_abs_cmd_y_max_ &&
          std::abs(cmd.cmdVelAngularZ_) < roll_compensation_closed_loop_abs_cmd_ang_z_max_;
      if (closed_loop_active)
      {
        const double roll_error_rad =
            roll_compensation_target_roll_rad_ - roll_compensation_filtered_roll_rad_;
        const double max_compensation_rad =
            roll_compensation_closed_loop_max_deg_ * M_PI / 180.0;
        if (roll_compensation_closed_loop_ki_ > 1e-9)
        {
          const double integral_limit =
              max_compensation_rad / roll_compensation_closed_loop_ki_;
          roll_compensation_integral_rad_sec_ = std::clamp(
              roll_compensation_integral_rad_sec_ + roll_error_rad * observation_dt,
              -integral_limit, integral_limit);
        }
        else
        {
          roll_compensation_integral_rad_sec_ = 0.0;
        }

        const double closed_loop_compensation_rad = std::clamp(
            roll_compensation_closed_loop_kp_ * roll_error_rad +
                roll_compensation_closed_loop_ki_ * roll_compensation_integral_rad_sec_,
            -max_compensation_rad, max_compensation_rad);
        total_roll_compensation_deg += closed_loop_compensation_rad * 180.0 / M_PI;
      }
    }

    if (std::abs(total_roll_compensation_deg) > 1e-6)
    {
      const double compensation_roll_rad = total_roll_compensation_deg * M_PI / 180.0;
      projected_gravity =
          Eigen::AngleAxisd(compensation_roll_rad, Eigen::Vector3d::UnitX()) *
          projected_gravity;
    }

    if (enable_back_arm_enhance_ && is_walking_mode &&
        cmd.cmdVelLineX_ >= -0.3 && cmd.cmdVelLineX_ <= -0.02)
    {
      projected_gravity =
          Eigen::AngleAxisd(2.7 * M_PI / 180.0, Eigen::Vector3d::UnitY()) * projected_gravity;
    }

    if (enable_standup_enhance_ && stand_up_rising_active_)
    {
      const double height_cmd = cmd.cmdVelAngularZ_;
      double pitch_weight = 1.0;
      if (height_cmd < 0.0)
      {
        pitch_weight = std::clamp(
            -height_cmd / std::abs(kStandUpPitchFadeHeightStart_),
            0.0, 1.0);
      }
      else
      {
        pitch_weight = 0.0;
      }
      const double pitch_deg = kStandUpGravityPitchBiasDeg_ * pitch_weight;
      if (std::abs(pitch_deg) > 1e-6)
      {
        projected_gravity =
            Eigen::AngleAxisd(pitch_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()) * projected_gravity;
      }
    }

    Eigen::VectorXd local_action = getCurrentAction();

    // === 3. 填充 singleInputData / networkInputDataRL_ ===
    std::map<std::string, Eigen::VectorXd> singleInputDataMap = {
        // old name:
        {"base_ang_vel", bodyAngVel},
        {"projected_gravity", projected_gravity},
        {"velocity_commands", velocity_commands},
        {"command_state", command_state},
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
        {"command_scalar_state", tempCommand_scalar_state},
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

      if (is_amp_hand_controller_ && lateral_elbow_fix_ && action.size() == 21)
      {
        CommandDataRL elbowCmd = gait_receiver_->getCurrentCommand();
        elbowCmd.scale();

        const bool external_arm_control_active = arm_command_replacement_enabled_ &&
                                                 jointArmNum_ > 0 && arm_controller_ &&
                                                 arm_controller_->getMode() != 1;
        if (elbowCmd.cmdVelLineX_ < 0.0)
        {
          if (is_amp_hand_controller_)
          {
            elbowCmd.cmdVelLineX_ = std::max(elbowCmd.cmdVelLineX_, -cmdVelLineXNeg_);
          }
          else
          {
            const double neg_scale = external_arm_control_active ? cmdVelLineXNegScaleExternalArm_
                                                                 : cmdVelLineXNegScale_;
            elbowCmd.cmdVelLineX_ *= neg_scale;
          }
        }

        const bool is_lateral_move_command =
            std::abs(elbowCmd.cmdVelLineX_) < 0.2 &&
            std::abs(elbowCmd.cmdVelAngularZ_) < 0.2 &&
            std::abs(elbowCmd.cmdVelLineY_) > 0.1;
        if (is_lateral_move_command)
        {
          // kuavo_v17 action order: zarm_l4_joint=16, zarm_r4_joint=20.
          // Positive cmd_y is left lateral, negative cmd_y is right lateral.
          if (elbowCmd.cmdVelLineY_ > 0.0)
          {
            action[0] -= 0.08;
            action[1] *= 0.6;
            action[2] *= 1.5;
            action[16] *= kLateralElbowFixScale_;
          }
          else
          {
            action[0] -= 0.03;
            action[7] *= 0.7;
            action[8] *= 1.2;
            action[20] *= kLateralElbowFixScale_;
          }
        }
      }

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
      
      // 更新站立状态（用于下次检测切换）
      lastStanceState_ = is_standing;
      
      // 注意：不在这里修改 action，保持 action 为 RL 原始输出用于观测
      // 手臂置零和平滑处理将在 updateRLcmd 中单独进行
      
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

    // 应用手臂接管平滑处理（站立时置零，行走时平滑过渡）
    CommandDataRL currentCmdData = gait_receiver_->getCurrentCommand();
    currentCmdData.scale();
    if (currentCmdData.cmdVelLineX_ < 0.0)
    {
      if (is_amp_hand_controller_)
      {
        currentCmdData.cmdVelLineX_ = std::max(currentCmdData.cmdVelLineX_, -cmdVelLineXNeg_);
      }
      else if (!arm_command_replacement_enabled_ || !jointArmNum_ || !arm_controller_ ||
               arm_controller_->getMode() == 1)
      {
        currentCmdData.cmdVelLineX_ *= cmdVelLineXNegScale_;
      }
      else
      {
        currentCmdData.cmdVelLineX_ *= cmdVelLineXNegScaleExternalArm_;
      }
    }
    const Eigen::VectorXd& active_default = getActiveDefaultJointPos(currentCmdData);
    bool is_standing = (currentCmdData.cmdStance_ >= 1.0);
    applyArmTakeoverBlend(local_action, ros::Time::now(), is_standing);

    if (enable_back_arm_enhance_)
    {
      CommandDataRL backArmCmd = currentCmdData;
      backArmCmd.scale();
      if (backArmCmd.cmdVelLineX_ < kBackArmEnhanceCmdXThreshold_)
      {
        const double arm1_scale_ratio = kBackArmEnhanceScale_ / actionScale_;
        local_action[13] = -0.75+1.0*local_action[10];
        local_action[13] *= arm1_scale_ratio;
        local_action[14] *= 0.5;
        local_action[16] += 1.5-0.8*local_action[4];
        local_action[17] = -0.75+1.0*local_action[4];
        local_action[17] *= arm1_scale_ratio;
        local_action[18] *= 0.5;
        local_action[20] += 1.5-0.8*local_action[10];
      }
    }

    if (enable_elbow_scale_)
    {
      const double elbow_scale_ratio = 0.18 / actionScale_;
      local_action[16] *= elbow_scale_ratio;
      local_action[20] *= elbow_scale_ratio;
    }

    const bool stand_up_rising_for_action =
        is_amp_hand_controller_ && currentCmdData.cmdStance_ >= 1.0 &&
        ((amp_mode_ == 1) || std::abs(smoothed_stance_height_cmd_) > 1e-6) &&
        (((amp_mode_ == 1) ? currentCmdData.cmdVelAngularZ_ : 0.0) - smoothed_stance_height_cmd_) >
            kStandUpHeightRisingEpsilon_;

    if (enable_standup_enhance_ && stand_up_rising_for_action)
    {
      static constexpr int kStandUpLeg4ActionIndices[] = {
          kStandUpLegL4ActionIdx_, kStandUpLegR4ActionIdx_};
      for (const int idx : kStandUpLeg4ActionIndices)
      {
        if (idx >= 0 && idx < local_action.size())
        {
          local_action[idx] *= kStandUpLeg4ActionScale_;
        }
      }
      if (kStandUpLegL1ActionIdx_ >= 0 && kStandUpLegL1ActionIdx_ < local_action.size())
      {
        const double action = local_action[kStandUpLegL1ActionIdx_];
        const double bias_weight = std::clamp(
            (action - kStandUpLeg1ActionBiasFadeMin_) /
                (kStandUpLeg1ActionBiasFadeMax_ - kStandUpLeg1ActionBiasFadeMin_),
            0.0, 1.0);
        local_action[kStandUpLegL1ActionIdx_] +=
            kStandUpLeg1ActionBiasMinAbs_ +
            (kStandUpLeg1ActionBiasMaxAbs_ - kStandUpLeg1ActionBiasMinAbs_) * bias_weight;
      }
      if (kStandUpLegR1ActionIdx_ >= 0 && kStandUpLegR1ActionIdx_ < local_action.size())
      {
        const double action = local_action[kStandUpLegR1ActionIdx_];
        const double bias_weight = std::clamp(
            (action - kStandUpLeg1ActionBiasFadeMin_) /
                (kStandUpLeg1ActionBiasFadeMax_ - kStandUpLeg1ActionBiasFadeMin_),
            0.0, 1.0);
        local_action[kStandUpLegR1ActionIdx_] -=
            kStandUpLeg1ActionBiasMinAbs_ +
            (kStandUpLeg1ActionBiasMaxAbs_ - kStandUpLeg1ActionBiasMinAbs_) * bias_weight;
      }
    }

    if (enable_elbow_scale_)
    {
      // zarm_l4_joint=16, zarm_r4_joint=20: final target (action*scale+default) must stay < -0.05
      static constexpr int kElbowActionIndices[] = {16, 20};
      static constexpr double kElbowTargetUpperBound = -0.15;
      for (const int idx : kElbowActionIndices)
      {
        const double scale = actionScale_ * actionScaleTestRL_[idx];
        if (scale <= 0.0)
          continue;
        const double max_action = (kElbowTargetUpperBound - active_default[idx]) / scale;
        if (local_action[idx] > max_action)
          local_action[idx] = max_action;
      }
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
    Eigen::VectorXd torque(jointNum_ + jointArmNum_ + waistNum_);// 策略理论计算扭矩
    for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
    {
      jointTor(i) = jointTor(i) + jointKpRL_(i) * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + active_default[i]);
    }
    if (is_real_)
    {
      for (int i = 0; i < jointNum_ + jointArmNum_ + waistNum_; i++)
      {
        if (JointControlModeRL_(i) == 0)
        {
          if (JointPDModeRL_(i) == 0)
          {
            cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + active_default[i]) - jointKdRL_[i] * jointVel[i];
            cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
            torque[i] = cmd[i];
          }
          else
          {
            cmd[i] = (local_action[i] * actionScale_ * actionScaleTestRL_[i] + active_default[i]);
            torque[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + active_default[i]) - jointKdRL_[i] * jointVel[i];
          }
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + active_default[i]);
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
          cmd[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + active_default[i]) - jointKdRL_[i] * jointVel[i];
        }
        else if (JointControlModeRL_(i) == 2)
        {
          cmd[i] = jointTor[i];
        }
        cmd[i] = std::clamp(cmd[i], -torqueLimitsRL_[i], torqueLimitsRL_[i]);
        torque[i] = jointKpRL_[i] * (local_action[i] * actionScale_ * actionScaleTestRL_[i] - jointPos[i] + active_default[i]) - jointKdRL_[i] * jointVel[i];
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
      ros_logger_->publishVector("/rl_controller/torque", torque);
      ros_logger_->publishVector("/rl_controller/actuation", actuation);
    }


    return actuation;
  }

  void AmpWalkController::applyArmTakeoverBlend(Eigen::VectorXd& action, const ros::Time& time, bool is_standing)
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

  Eigen::VectorXd AmpWalkController::getDefaultArmJointPos() const
  {
    return defalutJointPosRL_.segment(jointNum_ + waistNum_, jointArmNum_);
  }

  Eigen::VectorXd AmpWalkController::getArmActionScaleTest() const
  {
    return actionScaleTestRL_.segment(jointNum_ + waistNum_, jointArmNum_);
  }

  Eigen::VectorXd AmpWalkController::getCurrentArmJointPos(const SensorData& sensor_data) const
  {
    if (is_roban_)
    {
      return sensor_data.jointPos_.segment(waistNum_ + jointNum_, jointArmNum_);
    }
    return sensor_data.jointPos_.segment(jointNum_ + waistNum_, jointArmNum_);
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
    if (gait_receiver_)
    {
      if (is_roban_)
      {
        // amp_mode=1 时，将站立命令切到“复用行走指令”语义（x/y/yaw 三通道）。
        // 起身平滑未完成时继续复用，避免摇杆回正后高度命令瞬间归零。
        const bool stand_up_smoothing_active =
            is_amp_hand_controller_ && stance_height_stand_up_smoothing_enabled_ &&
            std::abs(smoothed_stance_height_cmd_) > 1e-6;
        gait_receiver_->setReuseWalkCommandInStance(amp_mode_ == 1 || stand_up_smoothing_active);
      }
    }

    gait_receiver_->update(time, baseStateRL_, feetPositionsRL_);
    
    if (ros_logger_ && feetPositionsRL_.size() >= 24)
    {
      ros_logger_->publishVector("/rl_controller/feet_positions", feetPositionsRL_);

      Eigen::Vector2d foot_heights = Eigen::Vector2d::Zero();
      Eigen::Vector3d left_foot_center = Eigen::Vector3d::Zero();
      Eigen::Vector3d right_foot_center = Eigen::Vector3d::Zero();
      for (int i = 0; i < 4; ++i)
      {
        left_foot_center += feetPositionsRL_.segment<3>(3 * i);
        right_foot_center += feetPositionsRL_.segment<3>(12 + 3 * i);
        foot_heights(0) += feetPositionsRL_(3 * i + 2);
        foot_heights(1) += feetPositionsRL_(12 + 3 * i + 2);
      }
      left_foot_center /= 4.0;
      right_foot_center /= 4.0;
      foot_heights /= 4.0;
      ros_logger_->publishVector("/rl_controller/feet_heights", foot_heights);

      const Eigen::Vector3d foot_center_diff_world = left_foot_center - right_foot_center;
      ros_logger_->publishValue("/rl_controller/feet_x_diff_world", foot_center_diff_world.x());
      if (baseStateRL_.size() >= 4)
      {
        const double yaw = baseStateRL_(3);
        const double foot_x_diff_body = foot_center_diff_world.x() * std::cos(yaw) +
                                        foot_center_diff_world.y() * std::sin(yaw);
        ros_logger_->publishValue("/rl_controller/feet_x_diff_body", foot_x_diff_body);
      }

      double min_height = feetPositionsRL_(2);
      for (int i = 1; i < 8; ++i)
      {
        min_height = std::min(min_height, feetPositionsRL_(3 * i + 2));
      }
      Eigen::Vector2d foot_lift_heights;
      foot_lift_heights << foot_heights(0) - min_height,
                            foot_heights(1) - min_height;
      ros_logger_->publishVector("/rl_controller/feet_lift_heights", foot_lift_heights);
    }
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
        
        // 获取默认手臂位置（根据机型选择正确的手臂起始索引）
        Eigen::VectorXd default_arm_pos;
        if (defalutJointPosRL_.size() >= jointNum_ + waistNum_ + jointArmNum_)
        {
          if (is_roban_)
          {
            // roban 机型：关节顺序为 腰 + 腿 + 手臂，手臂从 waistNum_ + jointNum_ 开始
            default_arm_pos = defalutJointPosRL_.segment(waistNum_ + jointNum_, jointArmNum_);
          }
          else
          {
            // 其他机型：关节顺序为 腿 + 腰 + 手臂，手臂从 jointNum_ + waistNum_ 开始
            default_arm_pos = defalutJointPosRL_.segment(jointNum_ + waistNum_, jointArmNum_);
          }
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
    if (is_amp_hand_controller_)
    {
      nh.setParam("/amp_hand_controller/cmdVelLineXlow", cmdVelLineXLow_);
      nh.setParam("/amp_hand_controller/cmdVelLineXup", cmdVelLineXUp_);
      nh.setParam("/amp_hand_controller/cmdVelLineXNeg", cmdVelLineXNeg_);
      nh.setParam("/amp_hand_controller/squatHeightMin", squatHeightMin_);
      nh.setParam("/amp_hand_controller/squatHeightMax", squatHeightMax_);
      nh.setParam("/amp_hand_controller/ampVRcmdvelLinearXLimit", ampVRcmdvelLinearXLimit_);
      nh.setParam("/amp_hand_controller/ampVRcmdvelLinearYLimit", ampVRcmdvelLinearYLimit_);
      nh.setParam("/amp_hand_controller/ampVRcmdvelLinearZLimit", ampVRcmdvelLinearZLimit_);
      nh.setParam("/amp_hand_controller/ampVRcmdvelAngularYAWLimit", ampVRcmdvelAngularYAWLimit_);
    }

    ROS_INFO("[%s] Updated /velocity_limits from controller config: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             name_.c_str(),
             limits_vec[0], limits_vec[1], limits_vec[2],
             limits_vec[3], limits_vec[4], limits_vec[5]);
  }

} // namespace humanoid_controller
