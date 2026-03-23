#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include "humanoid_controllers/LowPassFilter.h"
#include "humanoid_controllers/rl/armController.h"
#include "kuavo_solver/ankle_solver.h"
#include <openvino/openvino.hpp>
#include <memory>
#include <map>
#include <mutex>

namespace humanoid_controller
{
  class PerceptionWalkController : public RLControllerBase
  {
  public:
    PerceptionWalkController(const std::string& name,
                      const std::string& config_file,
                      ros::NodeHandle& nh,
                      ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    ~PerceptionWalkController() override = default;

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
    
    void heightscanCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

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

    // 速度命令限制
    Eigen::Vector4d velocityLimits_{Eigen::Vector4d::Zero()};

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

    // AMP
    LowPassFilter2ndOrder jointCmdFilter_;
    Eigen::VectorXd jointCmdFilterState_;
    bool use_jointcmd_filter_{false};  // 是否使用关节指令滤波，由 skw_rl_param.info 中 use_jointcmd_filter 配置

    // 手臂控制相关（可选功能）
    std::unique_ptr<ArmController> arm_controller_; ///< 手臂控制器（统一管理手臂插值和控制）
    double arm_max_tracking_velocity_{0.5}; ///< 手臂最大跟踪速度 (rad/s)，从配置文件加载
    double arm_tracking_error_threshold_{0.05}; ///< 手臂跟踪误差阈值 (rad)，从配置文件加载
    double arm_mode_interpolation_velocity_{1.0}; ///< 模式2的插值速度 (rad/s)，从配置文件加载
    bool arm_initialization_done_{false}; ///< 手臂初始化插值是否完成，完成后全程RL接管
    
    ros::Subscriber heightscanSub_;
    std::vector<double> heightscan_;
  private:
    void updatePhase(const ocs2::humanoid::CommandDataRL& cmd);
    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& measuredRbdState);
    
    // 手臂控制辅助函数
    void initArmControl(const std::string& urdf_path);
  };
}
