#pragma once

// C++ Standard Library
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>

// ROS Messages
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/robotHeadMotionData.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/lbBaseLinkPoseCmdSrv.h"
#include "kuavo_msgs/changeTorsoCtrlMode.h"
#include "kuavo_msgs/changeLbQuickModeSrv.h"
#include "kuavo_msgs/changeLbMpcObsUpdateModeSrv.h"
#include <leju_mobile_base_msgs/BaseCmdVelStatus.h>

// Third Party
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

// Project Headers
#include "humanoid_interface/common/Types.h"
#include "humanoid_interface/common/TopicLogger.h"
#include "kuavo_common/common/sensor_data.h"
#include "humanoid_wheel_interface/HumanoidWheelInterface.h"
#include "humanoid_wheel_interface/motion_planner/VelocityLimiter.h"
#include "humanoid_wheel_interface_ros/MobileManipulatorDummyVisualization.h"
#include "humanoid_wheel_wbc/WeightedWbc.h"
#include "humanoid_wheel_wbc/ContactForceWbc.h"
#include "humanoid_controllers/WaistKinematics.h"
#include "humanoid_controllers/ControlDataManager.h"
#include "humanoid_controllers/ArmTrajectoryInterpolator.h"
#include "humanoid_controllers/ArmContactForceEstimatorWheel.h"
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"
#include "humanoid_wheel_interface/filters/jointCmdLimiter.h"
#include "humanoid_controllers/DesiredForceManager.h"

// hardware params
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "kuavo_common/common/json_config_reader.hpp"

namespace humanoidController_wheel_wbc
{
  using namespace ocs2;
  using namespace humanoid;

  // optimizedState_wbc / optimizedInput_wbc 与 MPC state/input 同维 (stateDim, 轮臂 s62 为 21)。
  // 二者一一对应：input[i] 为 state[i] 的时间导数（关节角速度 / 底盘速度）。
  // 布局 (wheelWorldBasedMobileManipulator, kuavo_s62):
  //   [0] base_x          [1] base_y          [2] base_yaw
  //   [3] knee_pitch      [4] leg_pitch       [5] waist_pitch     [6] waist_yaw_link
  //   [7:13]  z_arm_l1..l7  左臂 7DoF
  //   [14:20] z_arm_r1..r7  右臂 7DoF
  // 注：info.armDim=18 含下肢 4 关节 + 双臂 14 关节；baseDim_=stateDim-armDim=3。

  struct JointTrajectory {
    vector_t pos;
    vector_t vel;
    vector_t tau;

    void initialize(size_t size) {
      pos = vector_t::Zero(size);
      vel = vector_t::Zero(size);
      tau = vector_t::Zero(size);
    }
  };
  class humanoidControllerWheelWbc
  {
  public:
    humanoidControllerWheelWbc() = default;
    ~humanoidControllerWheelWbc();
    bool init(ros::NodeHandle &controller_nh, bool is_nodelet_node = false);
    bool starting(const ros::Time &time);  // 返回true表示成功，false表示超时失败
    bool preUpdate(const ros::Time &time);
    bool preUpdateComplete() {return isPreUpdateComplete;}
    void update(const ros::Time &time, const ros::Duration &period);

  protected:
    // ========== 常量定义 ==========
    static constexpr double ARM_MODE_SWITCH_HOLD_DURATION = 0.2;  // 200ms = 0.2s
    static constexpr int MEDIAN_FILTER_WINDOW_SIZE = 11;  // 中值滤波窗口大小
    const std::string robotName_ = "mobile_manipulator";

    // ========== 初始化和配置相关函数 ==========
    void setupHumanoidWheelInterface(const std::string &taskFile, const std::string &urdfFile, 
                                   const std::string &libFolder);
    void setupMrt();
    void initMPC();
    void registerAllServices();
    bool setupCpuIsolation();  // 从ROS参数获取隔离CPU索引并设置线程亲和性

    // ========== 运动学计算相关函数 ==========
    void getEEPose(const vector_t& init_q, std::vector<Eigen::Vector3d>& ee_pos, std::vector<Eigen::Matrix3d>& ee_rot);
    void getTorsoPose(const vector_t& init_q, Eigen::Vector3d& torso_pos, Eigen::Matrix3d& torso_rot);
    // 由 MPC state 构建 target-pose 格式单点目标（FK 现场计算），initMPC 与软暂停 RESUMING 共用。
    // zeroBase=true 底盘段置零（启动语义）；false 取 state 底盘段（恢复锚定）
    vector_t buildTargetPoseFromState(const vector_t& state, bool zeroBase);
    void computeObservationFromSensorData(const SensorData& sensorData, const vector6_t& odomData);

    // ========== 关节控制相关函数 ==========
    void updateUserJointCmd(const ros::Time &time, vector_t& target_qpos, vector_t& target_qvel);
    void applyArmTrajectoryInterpolation(const ros::Time& time, int8_t lbMpcMode, const SensorData& sensorData,
                                         vector_t& target_qpos, vector_t& target_qvel);
    vector_t smoothTransition(const vector_t& current_pos, const vector_t& target_pos, double transition_duration = 1.0);
    vector_t interpolateArmTarget(scalar_t currentTime, const vector_t& currentArmState, const vector_t& newDesiredArmState, scalar_t maxSpeed);
    vector_t processArmControlModeSwitch(const ros::Time& time, const vector_t& current_qpos, const vector_t& target_qpos);

    // ========== 服务回调函数 ==========
    bool enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    bool changeArmCtrlModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    bool handleWaistIkService(kuavo_msgs::lbBaseLinkPoseCmdSrv::Request &req, kuavo_msgs::lbBaseLinkPoseCmdSrv::Response &res);
    bool enableLbArmQuickModeCallback(kuavo_msgs::changeLbQuickModeSrv::Request &req, 
                                      kuavo_msgs::changeLbQuickModeSrv::Response &res);
    bool enableVelControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool enableControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);  // /enable_control service 回调
    bool changeLbObsUpdateModeCallback(kuavo_msgs::changeLbMpcObsUpdateModeSrv::Request &req,
                                      kuavo_msgs::changeLbMpcObsUpdateModeSrv::Response &res);
    // VR 增量遥操作相关服务回调
    bool enableVrArmAccelTaskCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool enableArmTrajInterpCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool enableVrArmKpKdCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    // ======= 底盘急停保护相关函数 ========
    void baseCmdVelStatusCallback(const leju_mobile_base_msgs::BaseCmdVelStatus::ConstPtr& msg);
    void publishStopRobot();

    // ======= 硬件相关处理函数 =========
    void replaceDefaultEcMotorPdoGait(kuavo_msgs::jointCmd& jointCmdMsg);    // 替换EC_MASTER电机的kp/kd（从running_settings）

    // ======= 机器人初始动作相关函数 ========
    void performSimpleActions(const ros::Time &time);
    void initialPreTargetActions(const vector_t& startActions, const vector_t& preTargetActions, double desiredTime);

    // ======= 更新期望位姿的误差分析 ========
    void computeErrorMultiEeFromTargetAndData(const vector_t& targetState, 
                                              const vector_t& currentState);

    // ========== 工具函数 ==========
    /**
     * @brief 创建零位姿态 [0, 0, 0, 1, 0, 0, 0]
     * @return 7维向量：[位置(xyz), 四元数(wxyz)]
     */
    inline static vector_t createZeroPose() {
        vector_t pose = vector_t::Zero(7);
        pose[3] = 1.0;  // qw = 1，表示单位旋转
        return pose;
    }

    double lowpassFilter(double current_value, double last_filtered, double alpha = 0.2) {
        if (alpha <= 0.0 || alpha >= 1.0) {
            throw std::invalid_argument("alpha必须是0到1之间的数值");
        }
        return alpha * current_value + (1 - alpha) * last_filtered;
    }

    std::vector<double> medianFilter(const std::vector<double>& data, int window_size) {
        std::vector<double> filtered_data;
        int data_size = data.size();
        for (int i = 0; i < data_size; ++i) {
            int start = std::max(0, i - window_size / 2);
            int end = std::min(data_size, i + window_size / 2 + 1);
            std::vector<double> window(data.begin() + start, data.begin() + end);
            std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
            filtered_data.push_back(window[window.size() / 2]);
        }
        return filtered_data;
    }
    // ========== 机器人启动初始动作 ==========
    vector_t preTargetActions_;
    vector_t startActions_;
    double robotPreActionDesiredTime_ = 0.0;

    // ========== 坐标变换相关 ==========
    Eigen::Vector3d cmdVelWorldToBody(const Eigen::Vector3d& cmd_vel_world, double yaw);
    Eigen::Vector3d cmdVelBodyToWorld(const Eigen::Vector3d& cmd_vel_body, double yaw);
    // 发布 /move_base/base_cmd_vel 前的速度上下界限幅
    void clampBaseCmdVel(geometry_msgs::Twist& cmd) const;

    // ========== 期望力控制相关函数 ==========
    vector_t getDesiredContactForce();

    // ========== 基础配置变量 ==========
    ros::NodeHandle controllerNh_;
    bool is_real_{false};
    bool isPreUpdateComplete{false};
    double dt_ = 0.001;
    int robotVersion_ = 60;

    // ========== ROS通信相关 ==========
    // 控制数据管理器
    std::unique_ptr<ControlDataManager> control_data_manager_;
    
    // 发布者
    ros::Publisher cmdVelPub_;
    ros::Publisher velControlStatePub_;
    ros::Publisher enableControlStatePub_;  // latched topic，同步 CDM 和 RM 的 enable 状态
    ros::Publisher jointCmdPub_;
    ros::Publisher waistYawKinematicPublisher_;  // waist_yaw_link运动学计算位置发布器
    ros::Publisher lbLegTrajPub_;  // lb_leg_traj话题发布者，用于外部MPC模式下的VR躯干控制
    ros::Publisher stopRobotPub_;  // /stop_robot 话题发布者，用于底盘急停保护
    ros::Publisher resetToStatePub_;  // /mobile_manipulator_reset_to_state 发布者（3791 软暂停恢复时把冻结姿态发给 RM）
    
    // 日志
    humanoid::TopicLogger *ros_logger_{nullptr};

    // ========== 底盘急停保护相关 ==========
    ros::Subscriber baseCmdVelStatusSub_;  // 订阅 /move_base/base_cmd_vel_status 话题
    bool enable_base_emergency_stop_{true};  // 底盘急停保护开关，默认打开
    std::atomic<bool> base_emergency_triggered_{false};  // 底盘急停触发标志

    // ========== 机器人参数 ==========
    size_t baseDim_{0};
    size_t armNum_{0};
    size_t lowJointNum_{0};
    size_t headNum_{2};

    // ========== MPC相关 ==========
    std::shared_ptr<mobile_manipulator::HumanoidWheelInterface> HumanoidWheelInterface_;
    std::shared_ptr<mobile_manipulator::MobileManipulatorDummyVisualization> robotVisualizer_;
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
    mobile_manipulator::ManipulatorModelInfo manipulatorModelInfo_;
    std::shared_ptr<MRT_ROS_Interface> mrtRosInterface_;
    bool enable_mpc_{false};
    size_t plannedMode_{0};
    vector_t optimizedState_mrt_, optimizedInput_mrt_, optimizedState_mrt_limit_, optimizedInput_mrt_limit_;
    int8_t mpcObsUpdateMode_{3};  // mpc优化采用的反馈机制: 0: 全部反馈, 1: 屏蔽下肢电机反馈, 2: 屏蔽上肢电机反馈, 3: 同时屏蔽上下肢电机反馈
                                  // 屏蔽反馈时, 采用MPC输出的期望作为反馈
    double mpcDt_{0.01};
    double mpcFreq_{100};

    // ========== 状态估计 ==========
    SystemObservation observation_wheel_;
    ros::Time current_time_, last_time_;

    // ========== 全身控制 ==========
    std::shared_ptr<mobile_manipulator::WeightedWbc> wheel_wbc_;  // 基础WBC（WeightedWbc）
    std::shared_ptr<mobile_manipulator::ContactForceWbc> contact_force_wbc_;  // 接触力控制WBC
    std::shared_ptr<mobile_manipulator::VelocityLimiter> velLimiter_;  // 梯形插补加减速
    Eigen::Vector3d base_cmd_vel_max_{1.2, 1.2, 1.2};   // vx, vy, wz 上限 [m/s, m/s, rad/s]
    Eigen::Vector3d base_cmd_vel_min_{-1.2, -1.2, -1.2};  // vx, vy, wz 下限
    bool base_cmd_vel_limit_enable_{false};  // 是否启用 base_cmd_vel 速度限幅
    double base_cmd_vel_publish_rate_{50.0};  // /move_base/base_cmd_vel 发布频率上限 [Hz]，避免 500Hz 灌满底盘节点
    ros::Time last_cmd_vel_pub_time_;  // 上次发布 /move_base/base_cmd_vel 的时间，用于限频

    // ========== 期望力管理器 ==========
    std::unique_ptr<DesiredForceManager> desired_force_manager_;

    // ========== VR控制相关 ==========
    bool use_vr_control_{false};  // 是否启用VR控制
    bool prev_whole_torso_ctrl_{false};  // 上一次的全身控制模式状态
    ros::ServiceClient mpc_control_client_;  // MPC模式切换服务客户端
    ros::ServiceClient reset_cmd_vel_ruckig_client_;  // 重置cmdVel Ruckig规划器服务客户端
    ros::ServiceClient dispatch_mode_client_;  // 底盘调度模式服务客户端 (/move_base/set_dispatch_mode)
    std_srvs::SetBool reset_cmd_vel_ruckig_srv_;  // 重置cmdVel Ruckig规划器服务请求

    // ========== enable control ==========
    ros::ServiceServer enableControlServiceServer_;
    int reset_to_state_publish_cnt_{0};  // RESUMING 受理后剩余发布拍数（连续发布兜底 RM 订阅竞态）
    vector_t reset_state_to_publish_;    // 待发布给 RM 的冻结姿态（21 维 MPC state）

    // disable 期间记录 WBC 实际执行值（limit filter 后、WBC 输入前）。PAUSING 首拍
    // 由主循环冻结（记录 optimizedState_mrt_limit_ + 写回 CDM），PAUSED/RESUMING 期间
    // WBC 冻结输出、RESUMING 期间 MPC 观测强制为它（反向写回，与 RM 参考锚一致）。
    vector_t frozen_state_;
    std::atomic<bool> frozen_state_valid_{false};
    std::mutex frozen_state_mutex_;

    // 3791: MPC policy 访问闸门（防 MRT reset() 异步线程与主线程 rollout/可视化竞态）。
    // pauseResumeMpcNode/resetMpcNode 内部 detached 线程执行 MRT_BASE::reset()，会析构
    // active policy 对象；getPolicy() 返回裸引用，主线程"isPolicyUpdated 检查→使用"之间
    // reset 抢先析构 → 悬垂访问 GPF（曾崩于 publishOptimizedTrajectory）。
    // 与双足 resetting_mpc_state_（非 NORMAL 不碰 policy）同思想：disable/enable 受理置
    // false，主线程观察到 active policy 被清空（reset 完成）后才开门，期间不访问 MRT。
    std::atomic<bool> mpc_policy_gate_open_{true};

    // 3791: MPC pause/resume 请求标志（受理 → 主循环安全点转交）。
    // enableControlCallback 在 roscpp 服务线程执行，pauseResumeMpcNode 内部是对 MPC
    // 节点服务的 roscpp 同步调用（无超时），若在回调线程内直接阻塞会卡住整个服务
    // 队列（dispatch_mode 等后续服务全部排队）。改为受理时仅置标志，主循环在可视化
    // 块之后（无任何 in-flight rollout/getPolicy 的安全点）消费并发起调用。
    std::atomic<bool> mpc_pause_requested_{false};
    std::atomic<bool> mpc_resume_requested_{false};

    // 3791: 软暂停四状态机（新语义，disable 受理即发 enable=false，仅 IDLE 发 true）：
    //   IDLE     运行稳定。受理 disable → PAUSING
    //   PAUSING  disable 过渡：PAUSING 首拍主循环冻结记录+CDM 写回；250 帧到点 → PAUSED
    //   PAUSED   暂停稳定。WBC 冻结输出；受理 enable → RESUMING
    //   RESUMING enable 过渡：RESUMING 首拍构建 rst_target 并 resetMpcNode；期间 MPC 观测
    //   强制 = 冻结+odom（反向写回）；250 帧到点 → IDLE 并发布 enable=true
    // 过渡态固定 250 帧（0.5s @ 500Hz WBC 循环）、到点退出（不做收敛判据，先实测再调）。
    // 拒绝窗口内的请求简单拒绝，不入队。快速 toggle 由固定周期吸收，不再打穿状态机。
    enum class SoftPauseState { IDLE, PAUSING, PAUSED, RESUMING };
    // atomic：enableControlCallback（服务线程）入口也要检查（受理/拒绝矩阵见实现）
    std::atomic<SoftPauseState> soft_pause_state_{SoftPauseState::IDLE};
    // 过渡态按控制循环帧数计数（受理时置 0，主循环每帧 ++）：250 帧 = 0.5s @ 500Hz。
    // 计数发生在 update() 主循环内，与受理记录天然同源，避免墙钟/仿真时间基准错位（3791）。
    std::atomic<int> soft_pause_transition_cycles_{0};
    static constexpr int SOFT_PAUSE_TRANSITION_CYCLES = 250;  // [帧] 过渡态固定周期数
    // RESUMING 首拍 reset 保护：单周期内 frozen_state_valid_ 置 true 后不变 false，
    // RESUMING 仅查它会每帧重复 resetMpcNode（solver 无法收敛）。本 flag 在 enable
    // 受理时置 false、RESUMING 首拍 reset 后置 true（disable 受理会清 frozen_state_valid_
    // 触发重冻结，故仅单周期内成立）。
    std::atomic<bool> resuming_reset_done_{false};

    // ========== 平滑过渡相关 ==========
    bool is_transitioning_{false};  // 是否正在过渡
    double transition_start_time_{0.0};  // 过渡开始时间
    vector_t waist_transition_start_pos_{vector_t::Zero(4)};  // 腰部关节过渡起始位置
    vector_t waist_prev_target_pos_{vector_t::Zero(4)};  // 记录腰部关节上一次的目标位置

    // ========== 滤波相关 ==========
    std::vector<std::vector<double>> median_filter_history_;  // 中值滤波历史数据
    vector_t last_filtered_low_joint_pos_ = vector_t::Zero(4);
    vector_t arm_start_pos_ = vector_t::Zero(14);

    // ========== 运动学计算 ==========
    std::shared_ptr<humanoid_controller::WaistKinematics> waistKinematics_;

    // ========== 手臂轨迹控制 ==========
    bool use_arm_trajectory_control_{false};  // 是否使用轨迹控制
    int8_t quickMode_{0};  // 全身快速模式类型: 0-关闭, 1-下肢快, 2-上肢快, 3-上下肢快
    bool use_vel_control_{true};  // 是否使用速度控制
    bool prev_use_vel_control_{true};  // 上一次的速度控制状态，用于检测模式切换
    ros::Time last_reset_cmd_vel_ruckig_time_;  // 上次重置cmdVel Ruckig规划器的时间
    static constexpr double RESET_CMD_VEL_RUCKIG_INTERVAL = 0.5;  // 重置规划器的最小时间间隔
    int arm_trajectory_mode_{-1};  // 轨迹控制模式
    int prev_arm_trajectory_mode_{0};  // 上一次的轨迹控制模式
    bool isArmControlModeChanged_{false};  // 是否需要处理模式切换
    bool arm_mode_switch_hold_phase_{true};  // 是否处于200ms保持阶段
    double arm_move_spd_{15.0};  // 手臂移动速度
    double arm_mode_switch_start_time_{0.0};  // 模式切换开始时间
    vector_t init_arm_target_qpos_;
    bool enable_arm_traj_interpolator_{false};  // 手臂轨迹插补增强开关（默认关闭，保持旧行为）
    ArmTrajectoryInterpolator armTrajectoryInterpolator_;
    vector_t wbc_arm_raw_q_;
    vector_t wbc_arm_raw_v_;

    // ========== 运动学限制滤波相关 ==========
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  obsStateLimitFilterPtr_;    // observation.state 限制滤波
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  obsInputLimitFilterPtr_;    // observation.input 限制滤波
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  mrtStateLimitFilterPtr_;    // mrtState 限制滤波
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  mrtInputLimitFilterPtr_;    // mrtInput 限制滤波

    vector_t observationMaxVel_, observationMaxAcc_, observationMaxJerk_;         //  限制参数
    vector_t optimizedTrajMaxVel_, optimizedTrajMaxAcc_, optimizedTrajMaxJerk_;

    std::shared_ptr<mobile_manipulator::jointCmdLimiter> jointCmdLimiterPtr_;

    // ========== 手臂末端力估计器 ==========
    std::unique_ptr<ArmContactForceEstimatorWheel> arm_force_estimator_;
    // ========== 硬件使用参数相关 ==========
    HighlyDynamic::HumanoidInterfaceDrake *drake_interface_{nullptr};
    HighlyDynamic::JSONConfigReader *robot_config_;
    HighlyDynamic::KuavoSettings kuavo_settings_;

    // ========== 底盘运动模式相关 ==========
    bool baseCmdVelStatus_{true};
    ros::Subscriber base_cmd_vel_status_sub_;

  };

} // namespace humanoidController_wheel_wbc
