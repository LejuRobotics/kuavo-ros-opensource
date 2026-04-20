#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include "humanoid_controllers/LowPassFilter.h"
#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/rl/waistController.h"
#include "kuavo_solver/ankle_solver.h"
#include "kuavo_msgs/ExecuteArmAction.h"
#include <openvino/openvino.hpp>
#include <memory>
#include <map>
#include <mutex>

namespace humanoid_controller
{
  class DepthWalkController : public RLControllerBase
  {
  public:
    DepthWalkController(const std::string& name,
                      const std::string& config_file,
                      ros::NodeHandle& nh,
                      ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    ~DepthWalkController() override = default;

    bool initialize() override;
    bool loadConfig(const std::string& config_file) override;
    void reset() override;
    void pause() override;
    void resume() override;

    /**
     * @brief 检查控制器是否准备好退出
     * @return 如果姿态角>60度（倒地），返回true，请求退出控制
     */
    bool isReadyToExit() const override;

    /**
     * @brief 检查控制器当前是否处于 stance（站立）模式
     * @return 如果 cmdStance_ == 1 返回 true，否则返回 false
     */
    bool isInStanceMode() const override;

    /**
     * @brief 更新速度限制到rosparam（重写基类方法）
     * 使用从配置文件加载的velocityLimits_设置速度限制
     */
    void updateVelocityLimitsParam(ros::NodeHandle& nh) override;

  protected:

    void inferenceThreadFunc() override;
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

    void depthCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

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
    double gait_fre=1.0;              // 步频
    double gait_phase=0.0;            // 步相位
    double leg_bias = 0.5;          // 腿部偏置
    double stance_ratio = 0.5;
    double currentCycleTime_{0.6};
    Eigen::Vector3d net_linvel;
    int episodeLength_{0};
    Eigen::VectorXd commandPhase_;     // sin(phase), cos(phase)
    Eigen::VectorXd frePhase_;
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

    // 速度命令限制（正负方向独立设置）
    // 格式：[linear_x_pos, linear_x_neg, linear_y_pos, linear_y_neg, 
    //        linear_z_pos, linear_z_neg, angular_z_pos, angular_z_neg]
    Eigen::Matrix<double, 8, 1> velocityLimits_{Eigen::Matrix<double, 8, 1>::Zero()};

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
    AnkleSolver ankleSolver_;

    // 是否使用 AMP 专用 Ruiwo 手臂增益（由 skw_rl_param.info 中 use_amp_ruiwo_kpkd 配置）
    bool use_amp_ruiwo_kpkd_{false};

    // AMP
    LowPassFilter2ndOrder jointCmdFilter_;
    Eigen::VectorXd jointCmdFilterState_;
    bool use_jointcmd_filter_{false};  // 是否使用关节指令滤波，由 skw_rl_param.info 中 use_jointcmd_filter 配置

    // 手臂控制相关（可选功能，arm_controller_ 已移至 RLControllerBase）
    double arm_max_tracking_velocity_{0.5}; ///< 手臂最大跟踪速度 (rad/s)，从配置文件加载
    double arm_tracking_error_threshold_{0.05}; ///< 手臂跟踪误差阈值 (rad)，从配置文件加载
    double arm_mode_interpolation_velocity_{1.0}; ///< 模式2的插值速度 (rad/s)，从配置文件加载

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

    // Ruiwo 电机参数切换服务客户端（用于 AMP 手臂增益切换）
    ros::ServiceClient srv_change_motor_param_;

    ros::Subscriber depthSub_;
    std::vector<double> depth_;
  private:
    void updatePhase(const ocs2::humanoid::CommandDataRL& cmd);
    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& measuredRbdState);
    
    // 手臂控制辅助函数
    void initArmControl(const std::string& urdf_path);
    
    // 腰部控制辅助函数
    void initWaistControl();

    // 异步切换 Ruiwo 电机参数，避免在控制循环中阻塞
    void changeRuiwoMotorParamAsync(const std::string& param_name);
  };
}
