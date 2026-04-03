#pragma once

// C++ Standard Library
#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
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
#include "humanoid_controllers/WaistKinematics.h"
#include "humanoid_controllers/ControlDataManager.h"
#include "humanoid_controllers/ArmTrajectoryInterpolator.h"
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"
#include "humanoid_wheel_interface/filters/jointCmdLimiter.h"

// hardware params
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "kuavo_common/common/json_config_reader.hpp"

namespace humanoidController_wheel_wbc
{
  using namespace ocs2;
  using namespace humanoid;
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

    // ========== 运动学计算相关函数 ==========
    void getEEPose(const vector_t& init_q, std::vector<Eigen::Vector3d>& ee_pos, std::vector<Eigen::Matrix3d>& ee_rot);
    void getTorsoPose(const vector_t& init_q, Eigen::Vector3d& torso_pos, Eigen::Matrix3d& torso_rot);
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
    bool changeLbObsUpdateModeCallback(kuavo_msgs::changeLbMpcObsUpdateModeSrv::Request &req, 
                                      kuavo_msgs::changeLbMpcObsUpdateModeSrv::Response &res);

    // ======= 硬件相关处理函数 =========
    void replaceDefaultEcMotorPdoGait(kuavo_msgs::jointCmd& jointCmdMsg);    // 替换EC_MASTER电机的kp/kd（从running_settings）

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

    // ========== 坐标变换相关 ==========
    Eigen::Vector3d cmdVelWorldToBody(const Eigen::Vector3d& cmd_vel_world, double yaw);
    Eigen::Vector3d cmdVelBodyToWorld(const Eigen::Vector3d& cmd_vel_body, double yaw);

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
    ros::Publisher jointCmdPub_;
    ros::Publisher waistYawKinematicPublisher_;  // waist_yaw_link运动学计算位置发布器
    ros::Publisher lbLegTrajPub_;  // lb_leg_traj话题发布者，用于外部MPC模式下的VR躯干控制
    
    // 日志
    humanoid::TopicLogger *ros_logger_{nullptr};

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
    bool reset_mpc_{false};
    bool enable_mpc_{false};
    size_t plannedMode_{0};
    vector_t optimizedState_mrt_, optimizedInput_mrt_;
    int8_t mpcObsUpdateMode_{3};  // mpc优化采用的反馈机制: 0: 全部反馈, 1: 屏蔽下肢电机反馈, 2: 屏蔽上肢电机反馈, 3: 同时屏蔽上下肢电机反馈
                                  // 屏蔽反馈时, 采用MPC输出的期望作为反馈

    // ========== 状态估计 ==========
    SystemObservation observation_wheel_;
    ros::Time current_time_, last_time_;

    // ========== 全身控制 ==========
    std::shared_ptr<mobile_manipulator::WbcBase> wheel_wbc_;
    std::shared_ptr<mobile_manipulator::VelocityLimiter> velLimiter_;  // 梯形插补加减速

    // ========== VR控制相关 ==========
    bool use_vr_control_{false};  // 是否启用VR控制
    bool prev_whole_torso_ctrl_{false};  // 上一次的全身控制模式状态
    ros::ServiceClient mpc_control_client_;  // MPC模式切换服务客户端
    ros::ServiceClient reset_cmd_vel_ruckig_client_;  // 重置cmdVel Ruckig规划器服务客户端
    std_srvs::SetBool reset_cmd_vel_ruckig_srv_;  // 重置cmdVel Ruckig规划器服务请求

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

    // ========== 硬件使用参数相关 ==========
    HighlyDynamic::HumanoidInterfaceDrake *drake_interface_{nullptr};
    HighlyDynamic::JSONConfigReader *robot_config_;
    HighlyDynamic::KuavoSettings kuavo_settings_;
  };

} // namespace humanoidController_wheel_wbc
