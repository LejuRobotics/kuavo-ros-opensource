#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include "humanoid_controllers/LowPassFilter.h"
#include "humanoid_controllers/rl/ArmTakeoverBlender.h"
#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/rl/waistController.h"
#include "kuavo_solver/ankle/ankle_solver.h"
#include "kuavo_msgs/ExecuteArmAction.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include <openvino/openvino.hpp>
#include <memory>
#include <map>
#include <mutex>
#include <deque>

namespace humanoid_controller
{
  class AmpWalkController : public RLControllerBase
  {
  public:
    AmpWalkController(const std::string& name,
                      const std::string& config_file,
                      ros::NodeHandle& nh,
                      ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    ~AmpWalkController() override = default;

    bool initialize() override;
    bool loadConfig(const std::string& config_file) override;
    void reset() override;
    void pause() override;
    void resume() override;

    /**
     * @brief 是否请求退出当前 RL 模式（与 RLControllerBase 一致）
     * 当估计的 roll/pitch 绝对值超过约 60°（判为跌倒）时返回 true，供上层切出行走模式。
     * @return 判为跌倒时为 true
     */
    bool requestToExit() const override;

    /**
     * @brief 是否允许从本控制器切换走（与 RLControllerBase 一致）
     * 当步态接收器当前指令为 stance 站立（cmdStance_ ≥ 0.5，实现中与 AmpWalk 一致）时返回 true，便于站立静止时切回 MPC 等；行走中一般为 false。
     * @return stance 指令成立时为 true
     */
    bool isAllowToExit() const override;

    /**
     * @brief 更新速度限制到rosparam（重写基类方法）
     * 使用从配置文件加载的velocityLimits_设置速度限制
     */
    void updateVelocityLimitsParam(ros::NodeHandle& nh) override;

  protected:
    // 主循环：从 RLControllerBase::update 调用
    bool updateImpl(const ros::Time& time,
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    // 推理：和 humanoidController_rl.cpp::inference 逻辑一致
    bool inference(const Eigen::VectorXd& observation,
                   Eigen::VectorXd& action) override;

    // 观测构造：和 humanoidController_rl.cpp::updateObservation / updatePhase 一致
    void updateObservation(const Eigen::VectorXd& state_est,
                           const SensorData& sensor_data) override;

    // 将 RL 输出的 actuation 映射到 jointCmd（基本照 humanoidController_rl.cpp::update 里 RL 分支）
    void actionToJointCmd(const Eigen::VectorXd& actuation,
                          const Eigen::VectorXd& measuredRbdState,
                          kuavo_msgs::jointCmd& joint_cmd) override;

    // 若需要，可限制什么时候跑 RL 推理
    bool shouldRunInference() const override;

    void preprocessSensorData(SensorData& sensor_data) override;

    // 更新手臂指令（可选功能，用于替换jointCmdMsg中的手臂部分）
    bool updateArmCommand(const ros::Time& time,
                         const SensorData& sensor_data,
                         kuavo_msgs::jointCmd& joint_cmd) override;

    // 更新腰部指令（可选功能，用于替换jointCmdMsg中的腰部部分）
    bool updateWaistCommand(const ros::Time& time,
                         const SensorData& sensor_data,
                         kuavo_msgs::jointCmd& joint_cmd) override;

  private:
    // === 来自 humanoidController_rl.cpp::loadSettings 的关键参数 ===
    double dt_{0.002};                    // 控制周期，和 /wbc_frequency 一致
    double actionScale_{1.0};
    double clipActions_{1.0};
    bool withArm_{true};
    CommandDataRL initial_cmd_;

    // 步态周期/phase 相关
    double cycleTime_{0.6};
    double cycleTime_short_{0.4};
    double switch_ratio_{0.5};
    double phase_{0.0};
    double currentCycleTime_{0.6};
    int episodeLength_{0};
    Eigen::Vector2d commandPhase_{Eigen::Vector2d::Zero()};
    ModeNumber rl_plannedMode_{ModeNumber::SS};

    // 观测相关
    int frameStack_{1};
    int numSingleObs_{0};
    std::vector<std::string> singleInputDataKeys_;
    std::deque<Eigen::VectorXd> inputDeque_; // 用于多个历史的存储输入
    // key -> {startIdx, numIdx, obsScale}
    std::map<std::string, Eigen::Vector3d> singleInputDataID_;
    Eigen::VectorXd singleInputData_;
    // networkInputDataRL_ 已在 RLControllerBase 里定义

    // 速度命令限制（简化版：4 维统一上限 + X 负向单独缩放系数）
    // 格式：[linear_x, linear_y, linear_z, angular_z]
    Eigen::Matrix<double, 4, 1> velocityLimits_{Eigen::Matrix<double, 4, 1>::Zero()};
    double cmdVelLineXLow_{0.0};  ///< amp_hand_controller 手柄十字键下档 X 速度限制
    double cmdVelLineXUp_{0.0};   ///< amp_hand_controller 手柄十字键高档 X 速度限制
    double cmdVelLineXNeg_{0.45}; ///< amp_hand_controller cmd_x 负向速度限制 (m/s)
    double squatHeightMin_{-0.3};  ///< amp_hand_controller 手柄 posture 模式最大下蹲深度 (m)
    double squatHeightMax_{0.01};   ///< amp_hand_controller 手柄 posture 模式最大站起高度 (m)
    double ampVRcmdvelLinearXLimit_{0.60};   ///< amp_hand VR 前进最大速度 (m/s)
    double ampVRcmdvelLinearYLimit_{0.0};    ///< amp_hand VR 侧向最大速度 (m/s)
    double ampVRcmdvelLinearZLimit_{0.0};    ///< amp_hand VR 高度通道最大速度 (m/s)
    double ampVRcmdvelAngularYAWLimit_{0.9}; ///< amp_hand VR yaw 最大角速度 (rad/s)
    double cmdVelLineXNegScale_{1.0};  ///< X 负向单独缩放系数（用于实现不对称速度限制：neg_limit = limit * this_scale）
    double cmdVelLineXNegScaleExternalArm_{1.0};  ///< amp_hand 外部手臂控制时 X 负向缩放系数

    // yaw 对齐
    double my_yaw_offset_{0.0};

    // OpenVINO
    std::string networkModelPath_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    // gait 指令来源（替代 humanoidController_rl.cpp 中的 CommandData）
    std::unique_ptr<ocs2::humanoid::RlGaitReceiver> gait_receiver_;

    // 真实/机型配置
    bool is_real_{false};
    bool is_roban_{false};
    kuavo_solver::AnkleSolver ankleSolver_;


    // AMP
    LowPassFilter2ndOrder jointCmdFilter_;
    Eigen::VectorXd jointCmdFilterState_;
    bool use_jointcmd_filter_{false};  // 是否使用关节指令滤波，由 skw_rl_param.info 中 use_jointcmd_filter 配置

    // 手臂控制相关（可选功能，arm_controller_ 已移至 RLControllerBase）
    double arm_max_tracking_velocity_{0.5}; ///< 手臂最大跟踪速度 (rad/s)，从配置文件加载
    double arm_tracking_error_threshold_{0.05}; ///< 手臂跟踪误差阈值 (rad)，从配置文件加载
    double arm_mode_interpolation_velocity_{1.0}; ///< 模式2的插值速度 (rad/s)，从配置文件加载
    bool arm_rl_takeover_blend_enabled_{false}; ///< 是否启用站立外部手臂到行走RL手臂的平滑接管
    double arm_rl_takeover_blend_duration_{0.3}; ///< 手臂平滑接管时长（秒）
    bool arm_zero_action_in_standing_{false}; ///< 是否在站立状态下将RL手臂action输出置0
    bool last_stance_state_for_blend_{true}; ///< 用于手臂接管混合的站立状态跟踪
    ArmTakeoverBlender arm_takeover_blender_; ///< 站立到行走时的手臂RL接管混合器

    // 腰部控制相关（可选功能）
    double waist_mode_interpolation_velocity_{1.0}; ///< 腰部模式切换时的插值速度 (rad/s)，从配置文件加载，用于三次多项式插值
    double waist_mode2_cutoff_freq_{1.0}; ///< 腰部模式2外部输入的截止频率 (Hz)，从配置文件加载，默认5Hz
    Eigen::VectorXd waist_kp_from_config_; ///< 从配置文件读取的腰部 kp 参数
    Eigen::VectorXd waist_kd_from_config_; ///< 从配置文件读取的腰部 kd 参数
    std::unique_ptr<WaistController> waist_controller_; ///< 腰部控制器
    bool waist_zero_tracking_enabled_{false}; ///< 行走时是否启用腰部0位跟踪（忽略RL输出，强制跟踪默认位置）

    // 站立切换到行走时的支撑腿髋关节roll偏置参数
    double stanceToWalkHipRollBias_{0.0}; ///< 初始偏置值（弧度）
    double stanceToWalkBiasDuration_{0.0}; ///< 偏置衰减时间（秒）
    ros::Time stanceToWalkBiasStartTime_; ///< 偏置开始时间
    bool isStanceToWalkBiasActive_{false}; ///< 是否正在应用偏置
    int stanceToWalkBiasSupportLeg_{0}; ///< 支撑腿标识：-1左腿支撑，1右腿支撑

    // 状态跟踪（用于检测站立->行走切换）
    bool lastStanceState_{true}; ///< 上一帧是否站立

    // 髋关节pitch角度索引（预计算）
    int leftHipPitchIdx_{0};     ///< 左髋pitch关节索引（leg_l3_joint）
    int rightHipPitchIdx_{0};    ///< 右髋pitch关节索引（leg_r3_joint）

    // 髋关节pitch角速度数据收集（用于判断支撑腿）
    static constexpr double kHipPitchCollectionDuration_ = 0.08; ///< 髋关节pitch数据收集时间段（秒）
    static constexpr double kHipPitchVelIntegralThreshold_ = 0.0001; ///< 髋关节pitch角速度积分阈值
    ros::Time stanceToWalkHipPitchCollectionStartTime_; ///< 站立切换到行走的时间点
    double leftHipPitchVelIntegral_{0.0}; ///< 左髋pitch角速度累积积分值
    double rightHipPitchVelIntegral_{0.0}; ///< 右髋pitch角速度累积积分值
    bool isHipPitchDataCollected_{false}; ///< 是否已完成髋关节pitch数据收集

    // 髋关节action历史值（用于方向变化判断）- 保留用于其他逻辑
    double lastLeftHipAction_{0.0}; ///< 上一帧左髋关节action
    double lastRightHipAction_{0.0}; ///< 上一帧右髋关节action
    double lastActionDiffHip_{0.0}; ///< 上一帧左右髋关节action差值

    // 滑动窗口历史值（已废弃，保留用于兼容性）
    static const int kSlidingWindowSize = 5; ///< 滑动窗口大小
    std::deque<double> leftHipActionHistory_; ///< 左髋action历史队列
    std::deque<double> rightHipActionHistory_; ///< 右髋action历史队列

    // YAW补偿参数（用于旋转时X轴速度补偿）
    bool yaw_compensation_enabled_{false};        ///< 是否启用YAW补偿
    double yaw_compensation_x_bias_{0.0};         ///< 通用X轴偏置
    double yaw_compensation_threshold_{0.0};      ///< YAW阈值（角速度绝对值超过此值才补偿）
    double yaw_compensation_x_velocity_threshold_{0.01}; ///< X方向速度阈值
    bool yaw_compensation_separate_enabled_{false}; ///< 是否启用分开补偿（顺时针/逆时针）
    double yaw_compensation_x_bias_clockwise_{0.0};     ///< 顺时针旋转时X轴偏置
    double yaw_compensation_x_bias_counterclockwise_{0.0}; ///< 逆时针旋转时X轴偏置

    // amp_hand_controller 专用：外部手臂接管时 RL 使用虚拟手臂观测（仅 use_virtual_arm_obs 来自配置）
    bool is_amp_hand_controller_{false};
    bool use_virtual_arm_obs_{false};
    static constexpr double kVirtualArmObsPitchBaseDeg_{0.2};
    static constexpr double kVirtualArmObsPitchCompensationDeg_{1.5};
    static constexpr double kVirtualArmObsPitchBaseDegNeg_{0.5};
    static constexpr double kVirtualArmObsPitchCompensationDegNeg_{0.0};
    bool lateral_elbow_fix_{false};
    static constexpr double kLateralElbowFixScale_{0.5};
    bool enable_elbow_scale_{false};
    bool enable_back_arm_enhance_{false};
    static constexpr double kBackArmEnhanceScale_{0.22};
    static constexpr double kBackArmEnhanceCmdXThreshold_{-0.2};
    bool enable_standup_enhance_{false};
    static constexpr double kStandUpGravityPitchBiasDeg_{-5.0};   ///< 起身时 projected_gravity 后仰偏置 (deg)
    static constexpr double kStandUpPitchFadeHeightStart_{-0.05}; ///< 该高度以下按深度满偏置，趋近 0 时淡出 (m)
    static constexpr double kSquatPitchMaxCmd_{0.10};             ///< 深蹲时 cmdVelLineX_ 最大自动弯腰量 (m/s)
    static constexpr double kSquatPitchFadeHeightStart_{-0.02};   ///< 下蹲低于该高度才开始叠加弯腰 (m)
    static constexpr double kStandUpHeightRisingEpsilon_{1e-6};   ///< 高度命令上升判据阈值 (m)
    // defaultJointState 顺序：waist(0), leg_l1(1)..leg_l6(6), leg_r1(7)..leg_r6(12)
    static constexpr int kStandUpLegL1ActionIdx_{1};              ///< leg_l1_joint
    static constexpr int kStandUpLegL4ActionIdx_{4};              ///< leg_l4_joint（左膝）
    static constexpr int kStandUpLegR1ActionIdx_{7};              ///< leg_r1_joint
    static constexpr int kStandUpLegR4ActionIdx_{10};             ///< leg_r4_joint（右膝）
    static constexpr double kStandUpLeg4ActionScale_{1.1};        ///< 起身时 leg_l4/leg_r4 action 缩放
    static constexpr double kStandUpLeg1ActionBiasFadeMin_{-0.11}; ///< leg_l1/leg_r1 偏置 fade 区间下限
    static constexpr double kStandUpLeg1ActionBiasFadeMax_{-0.01}; ///< leg_l1/leg_r1 偏置 fade 区间上限，越接近 0 偏置越大
    static constexpr double kStandUpLeg1ActionBiasMinAbs_{0.05}; ///< leg_l1/leg_r1 最小偏置绝对值
    static constexpr double kStandUpLeg1ActionBiasMaxAbs_{0.1};   ///< leg_l1/leg_r1 最大偏置绝对值
    bool stand_up_rising_active_{false};                            ///< 本周期是否处于起身上升阶段
    bool enable_roll_compensation_{false};
    bool enable_off_cmdy_by_cmdx_{false};
    static constexpr double kOffCmdyByCmdXThreshold_{0.3};   ///< cmdx 超过该值时关闭 cmdy
    static constexpr double kOffCmdxByCmdYThreshold_{0.41};  ///< abs(cmdy) 超过该值时关闭 cmdx
    bool enable_off_cmdy_by_cmdangz_{false};
    static constexpr double kOffCmdyByCmdAngZThreshold_{0.3};   ///< abs(cmdangz) 超过该值时关闭 cmdy
    static constexpr double kOffCmdAngZByCmdYThreshold_{0.3};   ///< abs(cmdy) 超过该值时关闭 cmdangz
    bool tiny_cmdx_clip_enabled_{false};
    double tiny_cmdx_clip_pos_min_{0.0};  ///< 正向截断区间最小值
    double tiny_cmdx_clip_pos_max_{0.0};  ///< 正向截断区间最大值，区间内值截断为该值
    bool tiny_cmdy_clip_enabled_{false};
    double tiny_cmdy_clip_min_{0.0};  ///< abs(cmd_y) 截断区间最小值
    double tiny_cmdy_clip_max_{0.0};  ///< abs(cmd_y) 截断区间最大值，区间内值截断为该值
    bool tiny_cmd_angz_clip_enabled_{false};
    double tiny_cmd_angz_clip_min_{0.0};  ///< abs(cmd_angz) 截断区间最小值
    double tiny_cmd_angz_clip_max_{0.0};  ///< abs(cmd_angz) 截断区间最大值，区间内值截断为该值
    static constexpr double kRollCompensationCmdXThreshold_{0.2};
    static constexpr double kWalkingRollCompensationQuadA_{-0.2};
    static constexpr double kWalkingRollCompensationQuadB_{0.3};
    static constexpr double kWalkingRollCompensationQuadC_{-0.013};
    static constexpr double kTurnRollCompensationDeg_{-0.7};

    // AMP 模型模式（影响 command_state 第 0 维：0 纯 AMP 走路，1 站立/弯腰/下蹲动手，2 走路动手）
    int amp_mode_{0};
    ros::ServiceServer change_amp_mode_srv_;

    // 站立模式下下蹲后起身的高度命令平滑（仅 amp_hand_controller）
    bool stance_height_stand_up_smoothing_enabled_{true};
    double max_stance_height_stand_up_change_{0.004}; ///< 起身单步最大高度命令变化量 (m)
    double stance_height_smooth_start_{-0.1};         ///< 起身达到该高度前平滑，达到后直接跟随 (m)
    double max_stance_squat_depth_{0.16};             ///< 最大下蹲深度 (m)，高度命令下限为 -max_stance_squat_depth_
    double smoothed_stance_height_cmd_{0.0};          ///< 平滑后的下蹲高度命令

    // 下蹲守备（控制器端）：深蹲且转向较小时拦截姿态模式退出请求
    bool squat_posture_defense_enabled_{false};
    double squat_posture_defense_height_threshold_{-0.05};  ///< 平滑后高度命令低于该值视为深蹲 (m)
    double squat_posture_defense_abs_cmdangz_limit_{0.8};   ///< abs(cmdangz) 低于该值时与深蹲条件共同触发守备

    bool changeAmpModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                               kuavo_msgs::changeArmCtrlMode::Response &res);

    Eigen::VectorXd standDefaultJointPosRL_;
    bool use_stand_default_joint_state_{false};
    bool has_stand_default_joint_state_{false};
    double stand_velocity_threshold_{0.1};
    double stand_angular_velocity_threshold_{0.1};

  private:
    const Eigen::VectorXd& getActiveDefaultJointPos(const ocs2::humanoid::CommandDataRL& cmd) const;

    void updatePhase(const ocs2::humanoid::CommandDataRL& cmd);
    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& measuredRbdState);
    void applyArmTakeoverBlend(Eigen::VectorXd& action, const ros::Time& time, bool is_standing);
    Eigen::VectorXd getDefaultArmJointPos() const;
    Eigen::VectorXd getArmActionScaleTest() const;
    Eigen::VectorXd getCurrentArmJointPos(const SensorData& sensor_data) const;
    
    // 手臂控制辅助函数
    void initArmControl(const std::string& urdf_path);
    
    // 腰部控制辅助函数
    void initWaistControl();

    double applyTinyCmdxClip(double cmdx) const;
    double applyTinyCmdYClip(double cmdy) const;
    double applyTinyCmdAngzClip(double angz) const;
    void applyStanceHeightStandUpSmoothing(CommandDataRL& cmd);
    bool isSquatPostureDefenseActive() const;

  };
}
