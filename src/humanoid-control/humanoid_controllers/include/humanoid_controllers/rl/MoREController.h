#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include "humanoid_controllers/LowPassFilter.h"
#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/rl/waistController.h"
#include "humanoid_controllers/rl/ArmTakeoverBlender.h"
#include "kuavo_solver/ankle/ankle_solver.h"
#include "kuavo_msgs/ExecuteArmAction.h"
#include <kuavo_msgs/JoySticks.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/switchController.h>
#include <humanoid_plan_arm_trajectory/RobotActionState.h>
#include <openvino/openvino.hpp>
#include <std_msgs/Int32.h>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace humanoid_controller
{
  /**
   * @brief MoRE (Mixture-of-Residual-Experts) velocity policy controller for biped S54.
   *
   * ONNX inputs match ``ActorCriticResiMoE.as_onnx`` / ``export_more_policy_onnx.py``:
   * - ``observations``: (B, num_gait + policy_body_dim) — gait mixture one-hot first, then body obs.
   * - optional ``history``: (B, obs_history_len, policy_body_dim) — time-stacked body obs without gait prefix.
   */
  class MoREController : public RLControllerBase
  {
  public:
    MoREController(const std::string& name,
                   const std::string& config_file,
                   ros::NodeHandle& nh,
                   ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    ~MoREController() override = default;

    bool initialize() override;
    bool loadConfig(const std::string& config_file) override;
    void reset() override;
    void pause() override;
    void resume() override;
    bool requestToExit() const override;
    bool isAllowToExit() const override;
    void updateVelocityLimitsParam(ros::NodeHandle& nh) override;
    bool requestArmControlMode(int target_mode) override;

  protected:
    bool updateImpl(const ros::Time& time,
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    bool inference(const Eigen::VectorXd& observation,
                   Eigen::VectorXd& action) override;

    void updateObservation(const Eigen::VectorXd& state_est,
                           const SensorData& sensor_data) override;

    void actionToJointCmd(const Eigen::VectorXd& actuation,
                          const Eigen::VectorXd& measuredRbdState,
                          kuavo_msgs::jointCmd& joint_cmd) override;

    bool shouldRunInference() const override;
    void preprocessSensorData(SensorData& sensor_data) override;

    bool updateArmCommand(const ros::Time& time,
                          const SensorData& sensor_data,
                          kuavo_msgs::jointCmd& joint_cmd) override;

    bool updateWaistCommand(const ros::Time& time,
                            const SensorData& sensor_data,
                            kuavo_msgs::jointCmd& joint_cmd) override;

  private:
    double dt_{0.002};
    double policy_control_step_dt_{0.01};
    double actionScale_{1.0};
    double clipActions_{1.0};
    bool withArm_{true};
    ocs2::humanoid::CommandDataRL initial_cmd_;

    // MoRE policy layout (must match mjlab more_amp_env_cfg / export_more_policy_onnx)
    int num_gait_{3};
    int obs_history_len_{10};
    int policy_body_dim_{0};   ///< actor obs without leading gait mixture
    int policy_obs_dim_{0};    ///< num_gait + policy_body_dim (ONNX observations width)
    /// "command": 订阅 moreStyleCommandTopic（Int32 gate index）
    ///   numGait>=3: 0=pose, 1=walk_policy_arm, 2=walk_external_arm
    ///   numGait==2: 0=pose, 1=walk（手臂一律策略输出）
    /// "auto": 无速度->0; 有速度->1（三风格且有外部手臂->2）
    /// "fixed": 始终 defaultGaitStyleIndex
    std::string gait_style_mode_{"command"};
    /// motion_style_weights one-hot（index 0/1/2 = 风格1/2/3）:
    /// 风格1 [1,0,0] pose | 风格2 [0,1,0] walk+policy arm | 风格3 [0,0,1] walk+external arm
    int default_gait_style_index_{0};
    /// 仅当 motion style index == 该值时 posture_commands 非零；-1 表示不门控
    int posture_nonzero_gait_style_index_{-1};
    /// posture_commands[0] 高度增量 (m) 限幅
    double posture_height_min_{-0.3};
    double posture_height_max_{0.0};
    /// posture_commands[1] 弯腰/躯干俯仰增量 (rad) 限幅
    double posture_bend_min_{0.0};
    double posture_bend_max_{0.5};
    std::string style_command_topic_{"/more_motion_style_cmd"};
    int style_command_index_{0};
    mutable std::mutex style_command_mutex_;
    ros::Subscriber style_command_sub_;

    // Body observation (no gait prefix) — same keys/order as training actor tail
    int numSingleObs_{0};
    std::vector<std::string> singleInputDataKeys_;
    std::map<std::string, Eigen::Vector3d> singleInputDataID_;
    Eigen::VectorXd singleInputData_;
    std::deque<Eigen::VectorXd> history_frames_;

    Eigen::VectorXd policy_obs_;       ///< [gait_weights; body_obs] for ONNX
    Eigen::VectorXf onnx_obs_buffer_;  ///< float copy for OpenVINO (must outlive infer_request)
    Eigen::VectorXf history_tensor_;   ///< flattened (obs_history_len * policy_body_dim)

    Eigen::Matrix<double, 4, 1> velocityLimits_{Eigen::Matrix<double, 4, 1>::Zero()};
    double cmdVelLineXNegScale_{1.0};
    double my_yaw_offset_{0.0};
    bool yaw_offset_initialized_{false};

    std::string networkModelPath_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    bool onnx_actions_only_{true};
    bool log_policy_obs_{true};
    double log_policy_obs_throttle_sec_{0.0};
    bool has_obs_history_encoder_{true};

    std::unique_ptr<ocs2::humanoid::RlGaitReceiver> gait_receiver_;

    bool is_real_{false};
    bool is_roban_{false};
    kuavo_solver::AnkleSolver ankleSolver_;
    bool use_amp_ruiwo_kpkd_{false};

    LowPassFilter2ndOrder jointCmdFilter_;
    Eigen::VectorXd jointCmdFilterState_;
    bool use_jointcmd_filter_{false};

    double arm_max_tracking_velocity_{0.5};
    double arm_tracking_error_threshold_{0.05};
    double arm_mode_interpolation_velocity_{1.0};

    // 站立外部手臂到行走RL摆臂的平滑接管（与 AMP armRlTakeoverBlend 一致）
    bool arm_rl_takeover_blend_enabled_{false};
    double arm_rl_takeover_blend_duration_{0.3};
    bool arm_zero_action_in_standing_{false};
    ArmTakeoverBlender arm_takeover_blender_;
    bool last_stance_state_for_blend_{false};
    void applyArmTakeoverBlend(Eigen::VectorXd& action, const ros::Time& time, bool is_standing);

    // 下蹲/起身 posture 平滑过渡
    bool posture_smooth_enabled_{true};
    double posture_smooth_duration_{0.2};
    double smoothed_squat_{0.0};
    double squat_blend_start_{0.0};
    double squat_blend_target_{0.0};
    /// 上一次 computePostureCommands 时的 gait style，用于检测 style 回到 0 时清 squat
    int last_posture_style_index_{-1};
    /// 上一次 posture_commands.x() 原始值，用于检测 FSM 退出躯干控制时 posture→0
    double last_posture_raw_{0.0};

    double waist_mode_interpolation_velocity_{1.0};
    double waist_mode2_cutoff_freq_{1.0};
    Eigen::VectorXd waist_kp_from_config_;
    Eigen::VectorXd waist_kd_from_config_;
    std::unique_ptr<WaistController> waist_controller_;
    bool waist_zero_tracking_enabled_{false};

    bool yaw_compensation_enabled_{false};
    double yaw_compensation_x_bias_{0.0};
    double yaw_compensation_threshold_{0.0};
    double yaw_compensation_x_velocity_threshold_{0.01};
    bool yaw_compensation_separate_enabled_{false};
    double yaw_compensation_x_bias_clockwise_{0.0};
    double yaw_compensation_x_bias_counterclockwise_{0.0};

    // MoRE 手臂/上身控制模式切换（0/1/2，语义同 ArmController）
    int more_mode_{0};
    /// pose 风格下由 VR 手柄 X+A 设定的手臂模式；默认 1=策略/自然摆臂
    int pose_arm_control_mode_{1};  // 站立默认 RL 手臂自然放下，X+A 切换 1↔2
    /// VR 手柄 X+B 设定的手臂冻结标志；仅未冻结目标为 mode2 时允许保持冻结
    bool arm_frozen_{false};
    /// pose 风格下由 VR 手柄 X+A 设定的腰部模式；默认 2=外部控制（与手臂一致）
    int pose_waist_control_mode_{2};
    /// 进入站立前的行走风格：1=from style1, 2=from style2；决定站立→行走时切哪个style
    int stand_origin_style_{1};
    int last_synced_gait_style_index_{-1};
    kuavo_msgs::JoySticks quest_joystick_prev_;
    bool quest_joystick_initialized_{false};
    ros::Subscriber quest_joystick_sub_;
    ros::ServiceServer change_more_mode_srv_;
    ros::ServiceServer execute_arm_action_srv_;
    ros::ServiceClient system_arm_action_client_;

    // 动作播放状态追踪（用于播完恢复风格/手臂模式+走不停腿）
    int pre_action_style_{-1};
    int pre_action_arm_mode_{-1};
    bool action_pending_restore_{false};
    ros::Time action_start_time_;
    int skip_sync_after_restore_{0};
    ros::Time restore_time_;
    static constexpr double kActionTimeoutSec_ = 15.0;
    int last_robot_action_state_{-1};
    int robot_action_state_stable_count_{0};
    ros::Subscriber robot_action_state_sub_;
    void robotActionStateCallback(const humanoid_plan_arm_trajectory::RobotActionState::ConstPtr& msg);
    void checkAndRestoreAfterAction();

    ros::ServiceClient srv_change_motor_param_;

    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& measuredRbdState);
    int resolveMotionStyleIndex(const ocs2::humanoid::CommandDataRL& cmd) const;
    Eigen::VectorXd computeMotionStyleWeights(const ocs2::humanoid::CommandDataRL& cmd) const;
    Eigen::Vector2d computePostureCommands(int gait_style_index);
    void styleCommandCallback(const std_msgs::Int32::ConstPtr& msg);
    void initHistoryBuffer();
    void packHistoryTensor();
    /// 推理成功后 roll 并写入 body_t（供下一拍 cat(stored[1:], body) 使用）
    void commitHistoryFrame();
    void logPolicyObservation(const Eigen::VectorXd& motion_style_weights,
                              const Eigen::Vector3d& velocity_commands,
                              const Eigen::Vector2d& posture_commands) const;

    bool changeMoreModeCallback(kuavo_msgs::changeArmCtrlMode::Request& req,
                                kuavo_msgs::changeArmCtrlMode::Response& res);
    bool executeArmActionCallback(kuavo_msgs::ExecuteArmAction::Request& req,
                                   kuavo_msgs::ExecuteArmAction::Response& res);

    /// 当前 gate index（三风格 0/1/2；二风格 0=pose, 1=walk）
    int getCurrentGaitStyleIndex() const;
    /// 不考虑 X+B 冻结时，各 gait style 对应的基础手臂模式
    int resolveUnfrozenArmControlMode(int gait_style_index) const;
    /// numGait>=3 与现逻辑一致；numGait==2 时 walk(index 1) 固定策略手臂(1)
    int resolveArmControlMode(int gait_style_index) const;
    /// numGait>=3 与现逻辑一致；numGait==2 时 walk(index 1) 固定 RL 腰部(1)
    int resolveWaistControlMode(int gait_style_index) const;
    bool usesThreeGaitExperts() const { return num_gait_ >= 3; }
    bool hasExternalArmCommand() const;
    bool hasExternalWaistCommand() const;
    void syncWaistControllerMode(int gait_style_index, bool style_changed);
    void syncArmControllerMode(int gait_style_index);
    void questJoystickCallback(const kuavo_msgs::JoySticks::ConstPtr& msg);
    void handleQuestArmControlButtons(const kuavo_msgs::JoySticks& joy);
    /// use_external_arm_controller 且 ArmController 已成功初始化
    bool isArmControllerActive() const;

    void initArmControl(const std::string& urdf_path);
    void initWaistControl();
    void changeRuiwoMotorParamAsync(const std::string& param_name);

    /// 将 joint_cmd 中腰部与手臂的指令固定为 defaultJointState（joint_q=默认位姿，joint_v/tau=0）
    void applyUpperBodyDefaultJointCmd(kuavo_msgs::jointCmd& joint_cmd);
  };
}  // namespace humanoid_controller
