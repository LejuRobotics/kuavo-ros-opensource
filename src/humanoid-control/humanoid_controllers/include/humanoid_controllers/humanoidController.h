#pragma once

#include <atomic>
#include <memory>
#include <map>
#include <humanoid_interface/HumanoidInterface.h>
#include "humanoid_controllers/armTorqueController.h"
#include <controller_interface/controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <humanoid_common/hardware_interface/ContactSensorInterface.h>
#include "humanoid_controllers/sensor_data_types.h"
#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/FallStandController.h"
#include "humanoid_controllers/rl/RLControllerManager.h"

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include "humanoid_interface_ros/visualization/HumanoidVisualizer.h"
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <humanoid_estimation/StateEstimateBase.h>
#include <humanoid_wbc/WbcBase.h>

#include "humanoid_controllers/SafetyChecker.h"
#include "humanoid_controllers/visualization/humanoidSelfCollisionVisualization.h"
#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include <sensor_msgs/JointState.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include "kuavo_msgs/gaitTimeName.h"
#include "kuavo_msgs/switchController.h"
#include "kuavo_msgs/getControllerList.h"
#include "kuavo_msgs/switchToNextController.h"
#include "kuavo_msgs/robotWaistControl.h"
#include "kuavo_msgs/TransportModeCommand.h"

#include "humanoid_controllers/ArmTrajReceiver.h"

#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/jointCmd.h"
// #include "ocs2_biped_robot_ros/visualization/BipedRobotVisualizer.h"
#include "humanoid_interface/common/TopicLogger.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#ifdef KUAVO_CONTROL_LIB_FOUND
#include "kuavo_estimation/joint_filter/joint_filter.h"
#endif
#include "humanoid_common/hardware_interface/hardware_interface_ros.h"
#include <queue>
#include <mutex>
#include "humanoid_controllers/LowPassFilter.h"
#include "humanoid_interface/gait/GaitSchedule.h"
#include "kuavo_msgs/robotHeadMotionData.h"
#include "kuavo_msgs/getCurrentGaitName.h"
#include "kuavo_msgs/ExecuteArmAction.h"
#include "humanoid_controllers/shm_manager.h"
#include <std_msgs/Int8.h>
#if defined(USE_DDS) || defined(USE_LEJU_DDS)
#include "humanoid_controllers/CommonDDS.h"
#endif 

#include "kuavo_common/common/common.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <thread>
#include <functional>
#include <cmath>
#include <openvino/openvino.hpp>
#include <sensor_msgs/JointState.h>
#include "humanoid_controllers/rl/rl_switch_config.h"
#include "humanoid_controllers/ControllerCmdBlend.h"
#include "humanoid_controllers/LowPassFilter5thOrder.h"
#include "kuavo_solver/ankle/ankle_solver.h"
#include "humanoid_interface/foot_planner/floatInterpolation.h"
#include "humanoid_controllers/SitControlManager.h"
#include "humanoid_controllers/SitUpController.h"

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  
  // MotionTrajectoryData 已移动到 FallStandController.h 中

  struct gaitTimeName
  {
    std::string name;
    double startTime;
  };
  // SensorData 已移动到 sensor_data_types.h 中，直接使用即可

  enum ResettingMpcState
  {
    NORMAL = 0,
    RESET_INITIAL_POLICY,
    RESET_BASE,
  };

  enum FallStandState
  {
    STANDING = 0,
    FALL_DOWN
  };

  enum TransportModeState
  {
    TRANSPORT_INACTIVE = 0,
    TRANSPORT_INTERPOLATING,
    TRANSPORT_READY,       // 姿态到位，MPC 维持，关节未锁
    TRANSPORT_ACTIVE,      // CSP 锁死，可搬运
    TRANSPORT_HANDING_OVER // 移交管理权中（Joy 协调完成后 EXIT）
  }; // FALL_DOWN 不再作为状态，改用 transport_handoff_to_fallstand_ 标志位
  
  // struct SensorDataRL
  // {
  //   ros::Time timeStamp_;
  //   vector_t jointPos_;
  //   vector_t jointVel_;
  //   vector_t jointAcc_;
  //   vector_t jointCurrent_;
  //   vector3_t angularVel_;
  //   vector3_t linearAccel_;
  //   vector3_t freeLinearAccel_;
  //   Eigen::Quaternion<scalar_t> quat_;
  //   matrix3_t orientationCovariance_;
  //   matrix3_t angularVelCovariance_;
  //   matrix3_t linearAccelCovariance_;
  //   Eigen::Quaternion<scalar_t> quat_offset_;
  //   void resize_joint(size_t num)
  //   {
  //     jointPos_.resize(num);
  //     jointVel_.resize(num);
  //     jointAcc_.resize(num);
  //     jointCurrent_.resize(num);
  //   }
  // };
  class TrajectoryPublisher
  {
  public:
    TrajectoryPublisher(ros::NodeHandle &nh, double publish_time = 0.0010) : nh_(nh), desire_publish_time_(publish_time)
    { // 设置发布频率为1000Hz
      trajectory_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/humanoid_controller/policy/state_trajectory", 30);
      trajectory_thread_ = std::thread(&TrajectoryPublisher::publishThread, this);
    }

    ~TrajectoryPublisher()
    {
      if (trajectory_thread_.joinable())
      {
        trajectory_thread_.join();
      }
    }

    // 提供一个接口来传入轨迹进行发布
    void publishTrajectory(vector_array_t stateTraj)
    {
      std::lock_guard<std::mutex> lock(trajectory_queue_mutex_);
      trajectory_queue_.push(stateTraj);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub_;

    std::thread trajectory_thread_;
    std::queue<vector_array_t> trajectory_queue_;
    std::mutex trajectory_queue_mutex_;
    double publishing_rate_;
    double desire_publish_time_ = 0.0010;

    void publishThread()
    {

      while (ros::ok())
      {
        std::unique_lock<std::mutex> lock(trajectory_queue_mutex_);
        if (!trajectory_queue_.empty())
        {
          if (trajectory_queue_.size() > 4)
          {
            std::cout << "[TrajectoryPublisher]: queue size is too large: " << trajectory_queue_.size() << std::endl;
          }
          vector_array_t stateTraj = trajectory_queue_.front();
          trajectory_queue_.pop();
          lock.unlock();

          // 发布stateTraj中的状态信息，按照时间间隔进行发布
          const size_t numStates = stateTraj.size();
          for (size_t i = 0; i < numStates; ++i)
          {
            const auto &states = stateTraj.at(i);
            ros::Rate rate(states.size() / desire_publish_time_);
            std_msgs::Float32MultiArray msg;
            for (int i1 = 0; i1 < states.size(); ++i1)
            {
              msg.data.push_back(states(i1));
            }

            trajectory_pub_.publish(msg);

            rate.sleep();
          }
        }
        else
        {
          lock.unlock();
          std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
      }
    }
  };

  struct ArmJointTrajectory
  {
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd tau;
    void initialize(size_t num)
    {
      pos = Eigen::VectorXd::Zero(num);
      vel = Eigen::VectorXd::Zero(num);
      tau = Eigen::VectorXd::Zero(num);
    }
  };
  struct SystemObservationRL
  {
    size_t mode = 0;
    scalar_t time = 0.0;
    vector_t state;
    vector_t input;

    friend void swap(SystemObservationRL &a, SystemObservationRL &b) noexcept;
  };
  class humanoidController
  {
  public:
    humanoidController() = default;
    ~humanoidController();
    void keyboard_thread_func();
    bool init(HybridJointInterface *robot_hw, ros::NodeHandle &controller_nh, bool is_nodelet_node = false);
    bool preUpdate(const ros::Time &time);
    bool preUpdateComplete() {return isPreUpdateComplete;}
    void update(const ros::Time &time, const ros::Duration &period);
    void starting(const ros::Time &time);
    void stopping(const ros::Time & /*time*/) { mpcRunning_ = false; }

    /**
     * @brief 等待下一个控制周期（用于控制频率管理）
     * MPC 模式使用自身的 wbc_rate_，RL 模式委托给当前 RL 控制器
     */
    void waitForNextCycle();

    /**
     * @brief 获取当前控制器的控制频率
     * MPC 模式返回 wbc_frequency_，RL 模式返回当前 RL 控制器的频率
     * @return 控制频率（Hz）
     */
    double getControlFrequency() const;
    void applySensorData();
    void applySensorData(const SensorData &data);
    void applySensorDataRL(const SensorData &data);
    void updatakinematics(const SensorData &sensor_data, bool is_initialized_);
    void resetKinematicsEstimation();

    // MPC-RL 插值系统函数
    void startMPCRLInterpolation(double current_time, const vector6_t& target_torso_pose, const vector_t& target_arm_pos);
    void updateMPCRLInterpolation(double current_time);
    void startTransportMPCInterpolation();

    //waao： RL-RL插值
    // ==================== RL-RL插值系统函数声明 ====================
    void startRLToRLInterpolation(double current_time,
                                  const std::string& source_controller,
                                  const std::string& target_controller,
                                  const Eigen::VectorXd& start_joint_reference,
                                  const Eigen::VectorXd& target_joint_reference,
                                  const Eigen::VectorXd& start_joint_kp,
                                  const Eigen::VectorXd& start_joint_kd,
                                  const Eigen::VectorXd& start_joint_tau_max,
                                  const Eigen::VectorXd& start_joint_control_modes,
                                  const Eigen::VectorXd& start_joint_pd_modes,
                                  const Eigen::VectorXd& target_joint_kp,
                                  const Eigen::VectorXd& target_joint_kd,
                                  const Eigen::VectorXd& target_joint_tau_max,
                                  const Eigen::VectorXd& target_joint_control_modes,
                                  const Eigen::VectorXd& target_joint_pd_modes,
                                  const kuavo_msgs::jointCmd& target_joint_cmd);
    bool buildRLControllerJointCmd(RLControllerBase* controller,
                                   const ros::Time& time,
                                   kuavo_msgs::jointCmd& joint_cmd);
    //waao：相位对齐                               
    bool tryActivateRLToRLWalkingPhaseSync(RLControllerBase* source_controller,
                                           RLControllerBase* target_controller,
                                           double current_time);
    void refreshRLToRLSwitchParams();
    void updateRLToRLWalkingPhaseSync(double current_time);
    void clearRLToRLWalkingPhaseSync();
    void activateRLToRLLiveSourceController(RLControllerBase* source_controller,
                                            const std::string& source_controller_name,
                                            const std::string& target_controller_name,
                                            double current_time,
                                            const kuavo_msgs::jointCmd& target_joint_cmd);
    void applyRLToRLSwitchVelocityProfile(double current_time);
    void applyRLToRLLiveInterpolation(double current_time,
                                      const kuavo_msgs::jointCmd& source_joint_cmd,
                                      kuavo_msgs::jointCmd& target_joint_cmd);
    void applyRLToRLInterpolation(double current_time, kuavo_msgs::jointCmd& joint_cmd);
    void stopRLToRLInterpolation();
    void publishRLToRLSwitchDebugState(double progress, const std::string& phase, double current_time);

    sensor_msgs::Joy oldJoyMsg_;
    vector_t joystickOriginAxis_ = vector_t::Zero(6);
    vector_t joystickOriginAxisPre_ = vector_t::Zero(6);
    vector_t commadLineTarget_ = vector_t::Zero(6);
    double joystickSensitivity = 100;
    Eigen::VectorXd defalutArmPosMPC_;

    // param for RL
    Eigen::VectorXd currentDefalutJointPosRL_;
    Eigen::VectorXd JointControlModeRL_;
    Eigen::VectorXd JointControlModeStandRL_;
    Eigen::VectorXd JointPDModeRL_;
    Eigen::VectorXd initialStateRL_;
    Eigen::VectorXd jointCmdFilterStateRL_;
    Eigen::Vector3d accFilterStateRL_;
    Eigen::Vector3d freeAccFilterStateRL_;
    Eigen::Vector3d gyroFilterStateRL_;
    Eigen::Vector4d commandRL_;
    Eigen::Vector4d initalCommandRL_;
    Eigen::Vector4d commandScaleRL_;
    Eigen::VectorXd jointTorqueCmdRL_;
    Eigen::VectorXd jointKpRL_;
    Eigen::VectorXd jointKdRL_;
    Eigen::VectorXd torqueLimitsRL_;
    Eigen::VectorXd actionScaleTestRL_;
    Eigen::Vector4d velocityLimitsRL_;
    // int ankleSolverType_ = 0;
    double actionScaleRL_ = 0.25;
    int frameStackRL_ = 15;            // 多长时间步的obs
    int numSingleObsRL_ = 89;          // 单一时刻的obs的维度
    float cycleTimeRL_ = 1.2;          // 低步频周期时间
    float cycleTime_shortRL_ = 0.8;     // 高步频周期时间
    float currentCycleTimeRL_ = cycleTimeRL_;  // 当前周期值，初始化为普通周期
    float switch_ratioRL_ = 0.7;        //步频切换的比例值
    float phaseRL_ = 0;                // 初始相位设置为0
    int episodeLengthRL_ = 0;          // 用于计算相位，每走一个step就加1
    double clipObservationsRL_ = 18.0; // 用于对输入进行clip
    double clipActionsRL_ = 18.0;      // 用于对actions进行clip
    bool withArmRL_ = true;            // false: no arm, true: with arm
    double defaultBaseHeightControl_ = 0.9;
    double ruiwo_motor_velocities_factor_{0.0};
    std::string networkModelPath_;
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    std::unordered_map<std::string, double> scales_;                 // 存储obs的scale系数
    std::map<std::string, std::array<double, 3>> singleInputDataRLID_; // 存储singleInputData的id、min、max、scale
    std::vector<std::string> singleInputDataRLKeys;
    std::deque<Eigen::VectorXd> input_deque; // 用于多个历史的存储输入
    Eigen::VectorXd singleInputDataRL_;   // 单一时间的输入向量
    Eigen::VectorXd networkInputDataRL_; // 将所有历史长度的数据接成一个向量
    Eigen::VectorXd commandPhaseRL_;     // sin(phase), cos(phase)
    Eigen::VectorXd actionsRL_;
    double inferenceFrequencyRL_;
    std::mutex action_mtx_;
    std::mutex state_mtx_;
    std::mutex joint_cmd_mutex_;
    std::mutex joy_mutex_;
    std::mutex cmdvel_mutex_;
    std::thread inferenceThread_;
    Eigen::VectorXd joint_pos_limit; // 关节位置的限制
    Eigen::VectorXd joint_vel_limit; // 关节速度的限制

    
    MotionTrajectoryData motionTrajectory_;
    std::string trajectoryFilePath_;
    bool trajectoryLoaded_;
    bool residualAction_;

  protected:
    virtual void updateStateEstimation(const ros::Time &time, bool is_init = false);

    virtual void setupHumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitFile,
                                        bool verbose, RobotVersion rb_version);
    /** P3 终点：末端关节 + 脚底贴地（调 base_z）后由 RBD 转 centroidal */
    vector_t computeSeatOffsetCentroidalEnd(
        const humanoid_controller::SitControlManager::SeatJointTargets& end_targets,
        const vector_t& start_centroidal);
    /** P3：在预计算的 start/end centroidal 间按 α 插值，与关节偏置同 α */
    void recomputeOptimizedState2WbcWithSeatTargets(
        const humanoid_controller::SitControlManager::SeatJointTargets& targets);
    void clearSeatOffsetPlan();
    /** 落座 CSP 锁定姿释放后的起立入口：主类做 preUpdate 重入 / release hold / pause MPC，
     *  段0 规划与执行委托 SitUpController（prepare→activate）。 */
    bool beginStandUpFromSeat(std::string& err);
    /** 座椅起身 preUpdate 重入入口：重置 preUpdate 状态，暂停 MPC，委托 SitUpController 起立。
     *  统一 /bot_seat_return_preupdate 与 /humanoid_controller/seat_return_to_preupdate 两条触发路径。 */
    void beginSeatReturnToPreUpdate();
    bool seatReturnToPreUpdateCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void onSeatReturnPreUpdateTrigger(const std_msgs::Int8::ConstPtr& msg);
    bool seatStandUpStartCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void publishSeatReturnDone(bool ok);
    virtual void setupMpc();
    virtual void setupMrt();
    virtual void setupStateEstimate(const std::string &taskFile, bool verbose, const std::string &referenceFile);
    void sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg);
    void startMpccallback(const std_msgs::Bool::ConstPtr &msg);
    // void checkArmControlModeAndUpdateArmJoint();
    bool armJointSynchronizationCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    
    void robotlocalizationCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool tryApplyPendingExternalArmControllerMode();
    bool shouldBlockWalkingCommandForExternalArmTarget() const;
    bool enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    bool enableMmArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    bool getMmArmCtrlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    void real_init_wait();
    void publishHumanoidState(const vector_t& measuredRbdState);
    bool loadMotionTrajectory(const std::string &trajectoryFile);
    Eigen::VectorXd getTrajectoryCommand();
    Eigen::VectorXd getTrajectoryAnchorPos();
    Eigen::Quaterniond getTrajectoryAnchorQuat();
    Eigen::Vector3d getMotionAnchorPosB(const Eigen::Vector3d& currentBasePos, const Eigen::Quaterniond& currentBaseQuat);
    Eigen::VectorXd getMotionAnchorOriB(const Eigen::Quaterniond& currentBaseQuat);
 

    void swingArmPlanner(double st, double current_time, double stepDuration, Eigen::VectorXd &desire_arm_q, Eigen::VectorXd &desire_arm_v);
    void headCmdCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg);
    void fillHeadJointCmd(kuavo_msgs::jointCmd& msg, int head_start_index, bool push_back_mode);
    // 头部速度 / 相对位移接口（与躯干 /cmd_torso_vel、/cmd_torso_delta 对齐）
    // 开环绝对位姿只在 desire_head_pos_（rad），本类为唯一积分终点。
    // biped 无 soft-pause，无 enable guard。
    void headVelCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg);
    void headDeltaCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg);
    void applyHeadVelDeltaCommands();
    static vector_t clampHead2D(const vector_t& pose2d,
                                const std::vector<std::pair<double, double>>& limits_deg);
    void waistCmdCallback(const kuavo_msgs::robotWaistControl::ConstPtr &msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    // bool WalkenableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void visualizeWrench(const Eigen::VectorXd &wrench, bool is_left);
    bool getCurrentGaitNameCallback(kuavo_msgs::getCurrentGaitName::Request &req, kuavo_msgs::getCurrentGaitName::Response &res);
    void getEnableMpcFlagCallback(const std_msgs::Bool::ConstPtr &msg);
    
    void getEnableWbcFlagCallback(const std_msgs::Bool::ConstPtr &msg);
    void checkMpcPullUp(double current_time, vector_t & current_state, const TargetTrajectories& planner_target_trajectories);
#ifdef USE_DDS
    void LowStateCallback(const unitree_hg::msg::dds_::LowState_& data);
#elif USE_LEJU_DDS
    void LejuSensorsDataCallback(const leju::msgs::SensorsData& data);
#endif

    /**
     * Creates MPC Policy message.
     *
     * @param [in] primalSolution: The policy data of the MPC.
     * @param [in] commandData: The command data of the MPC.
     * @param [in] performanceIndices: The performance indices data of the solver.
     * @return MPC policy message.
     */
    static ocs2_msgs::mpc_flattened_controller createMpcPolicyMsg(const PrimalSolution &primalSolution, const CommandData &commandData,
                                                                  const PerformanceIndex &performanceIndices);


    void setJoyCmdState(const vector_t &joyCmdState)
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      joystickOriginAxis_ = joyCmdState;
    };
    void setCommandDataRL(const CommandDataRL &CommandDataRL)
    {
      std::lock_guard<std::mutex> lock(cmdvel_mutex_);
      CommandDataRL_ = CommandDataRL;
    };
    CommandDataRL getCommandDataRL()
    {
      std::lock_guard<std::mutex> lock(cmdvel_mutex_);
      return CommandDataRL_;
    };
    void setRobotState(const vector_t &state)
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      robotState_ = state;
    };
    vector_t getRobotState()
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      return robotState_;
    };
    void setRobotSensorData(const SensorData &sensor_data)
    {
      std::lock_guard<std::mutex> lock(sensor_data_mutex_);
      robotSensorsData_ = sensor_data;
    };
    SensorData getRobotSensorData()
    {
      std::lock_guard<std::mutex> lock(sensor_data_mutex_);
      return robotSensorsData_;
    };
    vector_t getJoyCmdState()
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      return joystickOriginAxis_;
    };

    ros::Time last_time_;
    ros::Time last_sensor_data_time_;
    ros::Time current_time_;
    std::queue<SensorData> sensorDataQueue;
    std::queue<nav_msgs::Odometry> robotlocalizationDataQueue;
    std::mutex sensor_data_mutex_;
    std::mutex robotlocalization_data_mutex_;

    std::thread keyboardThread_;
    int imuType_;

    // Interface
    std::shared_ptr<HumanoidInterface> HumanoidInterface_;
    std::shared_ptr<PinocchioInterface> pinocchioInterfaceWBCPtr_;
    std::shared_ptr<HumanoidInterface> HumanoidInterface_mpc;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsWBCPtr_;
    std::shared_ptr<PinocchioEndEffectorSpatialKinematics> eeSpatialKinematicsPtr_;
    std::shared_ptr<PinocchioInterface> pinocchioInterfaceEstimatePtr_;

    // State Estimation
    SystemObservation currentObservation_, currentObservationWBC_, lastObservation_;
    vector_t measuredRbdState_;
    vector_t measuredRbdStateRL_;
    vector_t measuredRbdStateReal_;
    vector_t simplifiedJointPos_;
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    std::shared_ptr<CentroidalModelRbdConversions> rbdConversionsWBC_;
    SensorData robotSensorsData_;
    SystemObservationRL currentObservationRL_, lastObservationRL_;
    vector_t robotState_;
    bool is_initialized_ = false;
    bool is_abnor_StandUp_{false};
    bool use_sit_init_{false};
    bool use_sit_init_boot_{false};
    bool wbc_only_{false};
    bool is_rl_controller_ = false;
    BufferedValue<bool> is_rl_controller_buffer_{false};
    bool is_mpc_controller_ = true;
    bool rl_available_ = false;  // RL参数文件是否存在，决定是否启用RL控制器功能
    bool is_rl_start_ = false;  // 如果为true，绕过MPC控制器，直接从rl_controllers.yaml的第一个控制器启动
    std::atomic_bool inference_running_{false};
    bool Walkenable_ = false;
    bool contactTrotgait_ = false;
    bool cmdTrotgait_ = false;
    bool cmdRLMode_ = false;                                         // RL模式命令标志
    // 跨线程触发: service 回调/keyboard 置位, 主循环 update 消费。原子避免数据竞争。
    std::atomic<bool> reset_mpc_{false};
    ResettingMpcState resetting_mpc_state_{ResettingMpcState::NORMAL};
    bool disable_mpc_{false};
    bool disable_wbc_{false};
    int hardware_status_ = 0;
    CommandDataRL initialCommandDataRL_;
    CommandDataRL CommandDataRL_;
    bool traj_start = false;
    double my_yaw_offset_;

    std::mutex disable_mpc_srv_mtx_;
    std::mutex disable_wbc_srv_mtx_;

    // Whole Body Control
    std::shared_ptr<WbcBase> wbc_;
    std::shared_ptr<SafetyChecker> safetyChecker_;
    std::shared_ptr<PinocchioEndEffectorSpatialKinematics> eeSpatialKinematicsWBCPtr_;

    // Nonlinear MPC
    std::shared_ptr<MPC_BASE> mpc_;
    std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;
    std::shared_ptr<MRT_ROS_Interface> mrtRosInterface_;

    //waitStandUpInit
    bool isRobotStandUpSuccess_;

    // preUpdate, 介于蹲姿启动和进MPC之间的状态处理
    double robotStartSquatTime_{0.0};
    double robotStartStandTime_{0.0};
    double robotStandUpCompleteTime_{0.0};
    bool is_robot_standup_complete_{false};
    bool isPreUpdateComplete{false};
    bool isInitStandUpStartTime_{false};
    /** preUpdate 起立首帧实测基座 [6..11]（CoM 位姿），避免与硬件 prep 后的真实姿态不一致 */
    bool isPullUp_{false};
    bool setPullUpState_{false};
    std::atomic<TransportModeState> transport_mode_state_{TRANSPORT_INACTIVE};
    std::atomic<bool> transport_handoff_to_fallstand_{false}; // FALL_DOWN 命令标志(主循环一帧消费)
    std::atomic<bool> transport_lock_pending_{false};       // LOCK 命令标志(主循环处理)
    std::atomic<bool> transport_reset_mpc_pending_{false};  // HAND_OVER 先对齐当前状态再插值(主循环处理)
    vector_t transport_target_pos_;  // ACTIVE 状态下的 CSP 目标关节位置(LOCK 时从当前位姿捕获)
    double transport_base_height_{0.7};  // 搬运躯干高度(m)，低于正常 MPC 站高以降低重心
    double standupTime_{0.0};
    double pull_up_trigger_time_{0.0};  // 拉起保护触发时间
    double arm_mode_sync_time_{0.0};  // 手臂模式同步完成的时间（当前模式切换到期望模式的时间）
    std::shared_ptr<WbcBase> standUpWbc_;
    std::shared_ptr<WbcBase> sitDownWbc_;
    std::string taskFile_switchParams_;
    vector_t curRobotLegState_;

    // Visualization
    std::shared_ptr<HumanoidVisualizer> robotVisualizer_;
    // std::shared_ptr<biped_robot::BipedRobotVisualizer> bipedRobotVisualizer_;//TODO: check if this is needed
    ros::Publisher observationPublisher_;
    ros::Publisher wbc_observation_publisher_;
    // Controller Interface
    ros::Publisher targetTorquePub_;
    ros::Publisher jointCmdPub_;
    ros::Publisher targetPosPub_;
    ros::Publisher targetVelPub_;
    ros::Publisher targetKpPub_;
    ros::Publisher targetKdPub_;
    ros::Publisher RbdStatePub_;
    ros::Publisher wbcFrequencyPub_;
    ros::Publisher wbcTimeCostPub_;
    ros::Publisher feettargetTrajectoriesPublisher_;
    ros::Publisher stop_pub_;
    ros::Publisher imuPub_;
    ros::Publisher kinematicPub_;
    ros::Publisher lHandWrenchPub_;
    ros::Publisher rHandWrenchPub_;
    ros::Publisher armEefWbcPosePublisher_;

    ros::Publisher standUpCompletePub_;
    ros::Subscriber jointPosVelSub_;
    ros::Subscriber sensorsDataSub_;
    ros::Subscriber robotLocalizationSub_;
    ros::Subscriber jointAccSub_;
    ros::Subscriber imuSub_;
    ros::Subscriber mpcStartSub_;
    ros::Subscriber observation_sub_;
    ros::Subscriber gait_scheduler_sub_;
    ros::Subscriber head_sub_;
    ros::Subscriber head_vel_sub_;
    ros::Subscriber head_delta_sub_;
    ros::Subscriber waist_sub_;
    ros::Subscriber head_array_sub_;
    ros::Subscriber mm_arm_joint_traj_sub_;
    ros::Subscriber arm_target_traj_sub_;//最终的手臂目标位置
    ros::Subscriber foot_pos_des_sub_;
    ros::Subscriber hand_wrench_sub_;
    ros::Subscriber arm_control_mode_sub_;
    ros::Subscriber contact_force_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::ServiceServer rl_control_service_;
    ros::Publisher stop_pub;
    ros::Publisher humanoidStatePublisher_;
    ros::Publisher mpcPolicyPublisher_;
    ros::Publisher cmdPoseWorldPublisher_; // 发布躯干位置控制命令 (geometry_msgs::Twist)


    ros::ServiceServer armJointSynchronizationSrv_;
    ros::Subscriber enable_mpc_sub_;
    ros::Subscriber enable_wbc_sub_;

    ros::ServiceServer enableArmCtrlSrv_;
    ros::ServiceServer enableMmArmCtrlSrv_;
    ros::ServiceServer getMmArmCtrlSrv_;
    ros::ServiceServer currentGaitNameSrv_;
    ros::ServiceServer triggerFallStandUpSrv_;
    ros::ServiceServer changeRuiwoMotorParamSrv_;
    ros::ServiceServer transport_mode_service_;
    GaitManager *gaitManagerPtr_=nullptr;

    PinocchioInterface *pinocchioInterface_ptr_;
    CentroidalModelInfo centroidalModelInfo_;
    CentroidalModelInfo centroidalModelInfoWBC_;
    Eigen::VectorXd hand_wrench_cmd_ = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd contactForce_ = Eigen::VectorXd::Zero(16);
    CentroidalModelInfo centroidalModelInfoEstimate_;

    // Node Handle
    ros::NodeHandle controllerNh_;
    HighlyDynamic::HumanoidInterfaceDrake *drake_interface_{nullptr};
    kuavo_solver::AnkleSolver ankleSolver;
#ifdef KUAVO_CONTROL_LIB_FOUND
    HighlyDynamic::JointFilter *joint_filter_ptr_{nullptr};
#endif
    HighlyDynamic::KuavoSettings kuavo_settings_;
    bool is_nodelet_node_{false};

    // 控制器列表相关
    std::vector<std::string> available_controllers_;
    std::string current_controller_;
    int current_controller_index_;

    void publishFeetTrajectory(const TargetTrajectories &targetTrajectories);

    bool transportModeCommandCallback(kuavo_msgs::TransportModeCommand::Request &req,
                                      kuavo_msgs::TransportModeCommand::Response &res);

    ros::ServiceServer real_initial_start_service_;  // /humanoid_controller/real_initial_start（仿真）
    ros::ServiceServer seat_return_to_preupdate_srv_; // /humanoid_controller/seat_return_to_preupdate
    ros::Publisher seat_return_preupdate_done_pub_;   // /bot_seat_return_preupdate_done
    ros::Subscriber sub_seat_return_preupdate_;       // /bot_seat_return_preupdate
    KuavoDataBuffer<SensorData> *sensors_data_buffer_ptr_;
    bool is_real_{false};
    bool is_cali_{false};
    char initial_input_cmd_ = '\0';
    // TrajectoryPublisher *trajectory_publisher_{nullptr};
    double dt_ = 0.001;
    std::thread mpcThread_;
    std::atomic_bool controllerRunning_{}, mpcRunning_{};

    // 控制频率相关
    double wbc_frequency_{500.0};                    // WBC 控制频率（Hz）
    std::unique_ptr<ros::Rate> wbc_rate_;            // WBC 控制频率 Rate 对象
    benchmark::RepeatedTimer mpcTimer_;
    benchmark::RepeatedTimer wbcTimer_;
    bool wbc_ran_this_frame_{false};                // 本帧 WBC 是否执行过（门控 time_cost 发布，防 DataAnalyzer 方差退化）
    size_t jointNum_ = 12;
    size_t armNum_ = 0;
    size_t headNum_ = 2;
    size_t waistNum_ = 1;
    size_t jointNumReal_ = 12;
    size_t armNumReal_ = 0;
    size_t actuatedDofNumReal_ = 12;// 实物的自由度
    size_t jointArmNum_ = 0;
    // vector_t desire_arm_q_prev_;
    // vector_t jointPos_, jointVel_;
    // vector_t jointAcc_;
    vector_t jointCurrent_;
    vector_t humanoidState_;
    ArmControlMode mpcArmControlMode_ = ArmControlMode::AUTO_SWING; // KEEP = 0, AUTO_SWING = 1, EXTERN_CONTROL = 2
    ArmControlMode mpcArmControlMode_desired_ = ArmControlMode::AUTO_SWING; // KEEP = 0, AUTO_SWING = 1, EXTERN_CONTROL = 2
    std::atomic_int arm_control_mode_desired_cache_{static_cast<int>(ArmControlMode::AUTO_SWING)};
    std::atomic_bool pending_external_arm_controller_mode_{false};
    std::atomic_bool more_activation_waiting_mode1_{false};
    double external_arm_default_position_tolerance_{0.05};
    double external_arm_target_hold_time_{0.5};
    Eigen::VectorXd external_arm_target_pos_;
    ros::Time last_external_arm_target_time_;
    bool has_external_arm_target_{false};
    mutable std::mutex external_arm_target_mutex_;
    int armDofMPC_ = 7; // 单手臂的自由度，会从配置文件中重新计算
    int armDofReal_ = 7; // 实际单手臂的自由度
    int armDofDiff_ = 0; // 单手臂的自由度差

    bool is_simplified_model_ = false;// 是否是简化的MPC模型
    TargetTrajectories currentArmTargetTrajectories_;// 当前手臂的目标轨迹，简化模型的关节target将会从这里读取
    int seq_ = 0;
    SensorData sensor_data_head_;
    SensorData sensor_data_headRL_;
    SensorData sensor_data_waist_;
    Eigen::Quaterniond robot_quat_state_update_;
    vector_t desire_head_pos_ = vector_t::Zero(2);
    // 头部 vel/delta 缓冲（rad/s, rad）；desire_head_pos_ 为唯一开环终点
    // vel: sticky + 0.3s 超时清零；delta: oneshot 用后即清
    static constexpr double kHeadVelTimeout_{0.3};
    vector_t cmd_head_vel_ = vector_t::Zero(2);    // [yaw, pitch] rad/s
    vector_t cmd_head_delta_ = vector_t::Zero(2);  // [d_yaw, d_pitch] rad oneshot
    bool is_cmd_head_vel_updated_{false};          // sticky
    bool is_cmd_head_vel_time_update_{false};      // 刷新 last_cmd_head_vel_time_
    bool is_cmd_head_delta_updated_{false};        // oneshot
    double last_cmd_head_vel_time_{0.0};
    double last_head_vel_integrate_time_{0.0};
    bool has_head_vel_integrate_time_{false};
    /** 进 MPC 站立后低头→0 插值（仅 keep_head_down 曾为 true；时长=head_raise_duration_seconds） */
    bool seat_head_raise_active_{false};
    double seat_head_raise_start_sec_{0.0};
    vector_t desire_waist_pos_ = vector_t::Zero(1);  // 腰部目标位置
    vector_t desire_arm_q_prev_;
    vector_t joint_pos_, joint_vel_, joint_acc_, joint_torque_;
    vector_t jointPosWBC_, jointVelWBC_, jointAccWBC_, jointCurrentWBC_;
    vector_t jointPosRL_, jointVelRL_, jointAccRL_, jointTorqueRL_;

    vector_t motor_c2t_;
    std::vector<std::vector<double>> motor_cul;
    std::vector<std::vector<double>> motor_coeff;
    bool init_input_ = false;
    Eigen::Quaternion<scalar_t> quat_;
    Eigen::Quaternion<scalar_t> quat_init;
    contact_flag_t contactFlag_;
    vector3_t angularVel_, linearAccel_, freeLinearAccel_;
    matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
    size_t plannedMode_ = ModeNumber::SS;
    size_t rl_plannedMode_ = ModeNumber::SS;
    size_t estPlannedMode_ = ModeNumber::SS;
    size_t nextMode_ = ModeNumber::SS;
    vector_t defalutJointPos_;
    vector_t initial_status_;
    vector_t initial_statusRL_;
    vector_t intail_input_;
    vector_t joint_kp_, joint_kd_, joint_kp_walking_, joint_kd_walking_, head_kp_, head_kd_, waist_kp_, waist_kd_;  // 添加腰部PD控制增益
    vector_t joint_kpRL_, joint_kdRL_, joint_kp_walkingRL_, joint_kd_walkingRL_, head_kpRL_, head_kdRL_, waist_kpRL_, waist_kdRL_;  // 添加腰部PD控制增益
    vector_t pull_up_status_;
    vector_t pull_up_input_;
    vector_t cur_status_;
    vector_t cur_input_;
    vector_t output_tau_, output_pos_, output_vel_;
    vector_t output_tauRL_, output_posRL_, output_velRL_;
    Eigen::MatrixXd joint_state_limit_; // 26x2, lower and upper limit
    double contact_cst_st_ = 0.1;
    double contact_cst_et_ = 0.1;
    double robotMass_ = 50;

    const std::string robotName_ = "humanoid";
    bool use_external_mpc_{true};
    bool use_joint_filter_{false};
    bool use_estimator_contact_{false};
    bool is_stance_mode_{false};
    bool is_roban_{false};
    bool only_half_up_body_{false};
    bool wheel_arm_robot_{false};
    bool stand_up_protect_{false};
    
    TopicLogger *ros_logger_{nullptr};
    vector_t optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_, initialState2WBC_mrt_, initialInput2WBC_mrt_;
    vector_t optimizedState_mrt_, optimizedInput_mrt_,stanceState_mrt_, stanceInput_mrt_;
    size_t optimized_mode_;
    bool is_play_back_mode_ = false;
    int control_mode_ = 2; // 0：CST, 1: CSV, 2:CSP
    LowPassFilter2ndOrder accFilterRL_;
    LowPassFilter2ndOrder freeAccFilterRL_;
    LowPassFilter2ndOrder gyroFilterRL_;
    LowPassFilter2ndOrder jointCmdFilterRL_;
    LowPassFilter5thOrder joystickFilterRL_;
    bool reset_estimator_ = false;
  bool reinitialize_controller_ = false;

    LowPassFilter2ndOrder acc_filter_;
    // LowPassFilter2ndOrder free_acc_filter_;
    LowPassFilter2ndOrder gyro_filter_;
    LowPassFilter2ndOrder arm_joint_pos_filter_;
    LowPassFilter2ndOrder arm_joint_vel_filter_;

    double sensor_frequency_{1000.0};   // 传感器数据频率
    double sensor_dt_{0.001};           // 传感器数据采样周期，用于滤波器和数据缓冲区
    LowPassFilter2ndOrder mrt_joint_vel_filter_;


    bool use_ros_arm_joint_trajectory_ = false;
    bool use_mm_arm_joint_trajectory_ = false;
    int ultra_fast_mode_;
    bool last_ultra_fast_mode_ = false;
    
    // ==================== MPC-RL插值系统成员变量 ====================
    std::shared_ptr<FloatInterpolation> torso_position_interpolator_ptr_; // 躯干位置插值器
    bool is_torso_interpolation_active_ = false;
    // 6D位姿插值的起点/目标/当前（xyz+rpy）
    vector6_t torso_interpolation_start_pose_;
    vector6_t torso_interpolation_target_pose_;
    vector_t leg_interpolation_start_pose_;
    vector_t leg_interpolation_target_pose_;
    double torso_interpolation_start_time_;
    double torso_interpolation_duration_; // 总期望插值时间 (s)
    double torso_interpolation_max_velocity_ = 0.1; // 最大插值速度 (m/s)
    double arm_interpolation_max_velocity_ = 1.0; // 最大插值速度 (rad/s)
    vector3_t default_torso_position_; // 默认躯干位置
    vector_t default_state_; // 基准状态
    vector6_t last_interpolated_pose_; // 上一次插值位姿
    double last_interpolation_time_; // 上一次插值时间
    TargetTrajectories mpc_current_target_trajectories_;
    ArmJointTrajectory arm_joint_trajectory_;
    ArmJointTrajectory mm_arm_joint_trajectory_;
    vector_t arm_joint_pos_cmd_prev_;
    vector_t joint_control_modes_;
    std::map<std::string, ModeSequenceTemplate> gait_map_;

    bool is_swing_arm_ = false;
    double swing_arm_gain_{0.0}, swing_elbow_scale_{0.0};
    // double ruiwo_motor_velocities_factor_{0.0};
    gaitTimeName current_gait_{"stance", 0.0}, last_gait_{"stance", 0.0};
    vector_t desire_arm_q, desire_arm_v;
    std::unique_ptr<ArmTorqueController> arm_torque_controller_;
    Eigen::VectorXd desire_arm_q_;
    Eigen::VectorXd desire_arm_v_;
    ros::Subscriber joint_sub_; // 添加订阅者成员变量
    bool is_standing_ = false;  // 添加站立状态标志位

    // Seat/Sit control manager
    std::unique_ptr<humanoid_controller::SitControlManager> sitControlManager_;
    /** 坐起身控制器：段0 reverse / await / 低头律。ROS 入口与 /bot_stand_up_complete 留主类。 */
    std::unique_ptr<humanoid_controller::SitUpController> sitUp_;
    vector_t seat_offset_centroidal_start_wbc_;
    vector_t seat_offset_centroidal_end_wbc_;
    bool seat_offset_centroidal_plan_ready_{false};

    // 共享内存通讯
    std::unique_ptr<gazebo_shm::ShmManager> shm_manager_;
    bool use_shm_communication_{false};  // 是否使用共享内存通讯
    bool updateSensorDataFromShm();      // 从共享内存更新传感器数据
    void publishJointCmdToShm(const kuavo_msgs::jointCmd& jointCmdMsg);         // 发布关节命令到共享内存
    void publishControlCommands(kuavo_msgs::jointCmd& jointCmdMsg);             // 发布控制命令的统一接口
    void replaceDefaultEcMotorPdoGait(kuavo_msgs::jointCmd& jointCmdMsg);                // 替换EC_MASTER电机的kp/kd（从running_settings）
    bool changeRuiwoMotorParamCallback(kuavo_msgs::ExecuteArmActionRequest &req, kuavo_msgs::ExecuteArmActionResponse &res);  // 修改ruiwo电机kp/kd，更新running_settings后由replaceDefaultEcMotorPdoGait生效

    // 手臂轨迹接收器（ROS topic + 增量 VR SHM 双通道）
    ArmTrajReceiver arm_traj_receiver_;

    // CPU内核隔离设置
    bool setupCpuIsolation();  // 从ROS参数获取隔离CPU索引并设置线程亲和性
    
    // 传感器数据发布
    ros::Publisher sensor_data_raw_pub_;
    feet_array_t<vector3_t> foot_pos_desired_;
    bool visualizeHumanoid_ = true;
    double timeout_warning_ms_ = 1000;
    double pull_up_force_threshold_ = 0.70;
    bool enable_pull_up_protect_ = false;

    std::vector<std::pair<double, double> > head_joint_limits_ = {{-80, 80}, {-25, 25}};
    
    // Unitree DDS hardware interface
#ifdef USE_DDS
    using HumanoidDDSClientType = HumanoidControllerDDSClient<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>;
    std::unique_ptr<HumanoidDDSClientType> dds_client_;
#elif defined(USE_LEJU_DDS)
    using HumanoidLejuDDSClientType = HumanoidControllerDDSClient<leju::msgs::JointCmd, leju::msgs::SensorsData>;
    std::unique_ptr<HumanoidLejuDDSClientType> dds_client_;
#endif
    
    // Latest sensor data for comparison
    SensorData latest_dds_sensor_data_;
    SensorData latest_ros_sensor_data_;
    bool has_dds_data_ = false;
    bool has_ros_data_ = false;
    std::vector<std::pair<double, double> > waist_joint_limits_ = {{-120, 120}};  // 添加腰部关节限制

    void publishWbcArmEndEffectorPose();

    // ==================== RL推理相关成员变量 ====================

    
    // ==================== 手臂插值相关成员变量 ====================
    // 手臂动作平滑插值相关变量
    int last_cmdStance_ = -1;        // 记录上一次的cmdStance_状态
    bool is_arm_interpolating_ = false; // 是否正在进行插值
    bool emergency_stop_interpolation_ = false; // 紧急停止插值标志
    double interpolation_start_time_ = 0.0; // 插值开始时间
    double interpolation_duration_ = 1.0;   // 插值持续时间（秒）
    Eigen::VectorXd arm_interpolation_start_pos_;  // 插值开始时的手臂位置
    Eigen::VectorXd arm_interpolation_start_vel_;  // 插值开始时的手臂速度
    Eigen::VectorXd arm_interpolation_target_pos_; // 插值目标位置
    Eigen::VectorXd arm_interpolation_target_vel_; // 插值目标速度
    bool last_is_rl_controller_ = false;

    bool has_fall_stand_controller_{false};
    
    //waao： RL-RL插值
    // ==================== RL-RL插值系统成员变量 ====================
    bool is_rl_to_rl_interpolation_active_ = false;
    bool has_last_rl_joint_reference_ = false;
    bool has_last_rl_joint_cmd_ = false;
    double rl_to_rl_switch_start_time_ = 0.0;
    double rl_to_rl_switch_duration_ = RL_TO_RL_SWITCH_DURATION_DEFAULT;
    std::string last_rl_controller_name_;
    std::string rl_to_rl_source_controller_name_;
    std::string rl_to_rl_target_controller_name_;
    RLControllerBase* rl_to_rl_live_source_controller_ptr_ = nullptr;
    Eigen::VectorXd last_rl_joint_reference_;
    kuavo_msgs::jointCmd last_rl_joint_cmd_;
    Eigen::VectorXd rl_to_rl_start_joint_reference_;
    Eigen::VectorXd rl_to_rl_target_joint_reference_;
    Eigen::VectorXd rl_to_rl_start_joint_q_msg_;
    Eigen::VectorXd rl_to_rl_start_joint_v_msg_;
    Eigen::VectorXd rl_to_rl_start_joint_tau_msg_;
    Eigen::VectorXd rl_to_rl_start_joint_tau_ratio_msg_;
    Eigen::VectorXd rl_to_rl_start_joint_kp_;
    Eigen::VectorXd rl_to_rl_start_joint_kd_;
    Eigen::VectorXd rl_to_rl_start_joint_tau_max_;
    Eigen::VectorXd rl_to_rl_start_joint_control_modes_;
    Eigen::VectorXd rl_to_rl_start_joint_pd_modes_;
    Eigen::VectorXd rl_to_rl_target_joint_kp_;
    Eigen::VectorXd rl_to_rl_target_joint_kd_;
    Eigen::VectorXd rl_to_rl_target_joint_tau_max_;
    Eigen::VectorXd rl_to_rl_target_joint_control_modes_;
    Eigen::VectorXd rl_to_rl_target_joint_pd_modes_;
    bool rl_to_rl_walking_phase_sync_active_ = false;
    RLControllerBase* rl_to_rl_phase_sync_source_controller_ptr_ = nullptr;
    RLControllerBase* rl_to_rl_phase_sync_target_controller_ptr_ = nullptr;
    double rl_to_rl_phase_sync_phi_rad_ = 0.0;
    double rl_to_rl_phase_sync_frequency_hz_ = 0.0;
    double rl_to_rl_phase_sync_source_frequency_hz_ = 0.0;
    double rl_to_rl_phase_sync_target_frequency_hz_ = 0.0;
    double rl_to_rl_phase_sync_delta_phi_rad_ = 0.0;
    double rl_to_rl_phase_sync_last_update_time_ = 0.0;
    double rl_to_rl_phase_sync_phi_thresh_rad_ = RL_TO_RL_PHASE_SYNC_PHI_THRESH_RAD_DEFAULT;
    double rl_to_rl_phase_sync_sin_guard_ = RL_TO_RL_PHASE_SYNC_SIN_GUARD_DEFAULT;
    double rl_to_rl_phase_sync_k0_ = RL_TO_RL_PHASE_SYNC_K0_DEFAULT;
    double rl_to_rl_phase_sync_decay_power_ = RL_TO_RL_PHASE_SYNC_DECAY_POWER_DEFAULT;
    double rl_to_rl_phase_sync_frequency_min_hz_ = RL_TO_RL_PHASE_SYNC_FREQUENCY_MIN_HZ_DEFAULT;
    double rl_to_rl_phase_sync_frequency_max_hz_ = RL_TO_RL_PHASE_SYNC_FREQUENCY_MAX_HZ_DEFAULT;
    bool rl_to_rl_velocity_dip_enabled_ = RL_TO_RL_VELOCITY_DIP_ENABLED_DEFAULT != 0;
    double rl_to_rl_velocity_dip_min_scale_ = RL_TO_RL_VELOCITY_DIP_MIN_SCALE_DEFAULT;
    double rl_to_rl_velocity_dip_midpoint_ = RL_TO_RL_VELOCITY_DIP_MIDPOINT_DEFAULT;

    ControllerCmdBlend cmd_blend_;
    // RL→MPC：cmd_blend 结束后再启动躯干插值（目标复用 torso/arm_interpolation_target_*）
    bool pending_rl_to_mpc_torso_interp_{false};

    // ==================== 通用插值系统成员变量 ====================
    std::mutex interpolation_mutex_;                                      // 插值任务的线程安全锁
    int interpolation_counter_;                                           // 插值任务计数器，用于生成唯一ID
    
    // ==================== 速度平滑系统成员变量 ====================
    geometry_msgs::Twist smoothed_cmd_vel_;                              // 平滑后的速度命令
    geometry_msgs::Twist previous_cmd_vel_;                              // 上一次的速度命令
    double velocity_smooth_factor_ = 0.1;                                // 速度平滑因子 (0.1表示每次更新10%的差值)
    double max_velocity_change_ = 0.5;                                   // 最大速度变化阈值
    double velocity_smooth_time_ = 0.1;                                  // 速度平滑时间窗口
    ros::Time last_velocity_update_time_;                                // 上次速度更新时间
    
    // ==================== 原地踏步系统成员变量 ====================
    bool is_in_place_stepping_ = false;                                  // 是否正在进行原地踏步
    ros::Time in_place_step_start_time_;                                 // 原地踏步开始时间
    double in_place_step_duration_ = 1.0;                                // 原地踏步持续时间（秒）
    geometry_msgs::Twist in_place_step_velocity_;                        // 原地踏步时的速度命令
    bool enable_in_place_stepping_ = false;                              // 是否启用原地踏步功能
    // 临时覆盖原地踏步持续时间（用于插值同步触发两秒原地踏步）
    bool temp_in_place_duration_override_active_ = false;
    double temp_in_place_duration_backup_ = 0.0;
    ros::Time temp_in_place_end_time_;
    
    // ==================== 持续原地踏步系统成员变量 ====================
    bool is_continuous_in_place_stepping_ = false;                       // 是否正在进行持续原地踏步
    geometry_msgs::Twist continuous_in_place_step_velocity_;             // 持续原地踏步时的速度命令
    
    // ==================== 站立键平滑过渡系统成员变量 ====================
    bool is_stance_transition_ = false;                                  // 是否正在进行站立过渡
    ros::Time stance_transition_start_time_;                             // 站立过渡开始时间
    double stance_transition_duration_ = 2.0;                            // 站立过渡持续时间（秒）
    geometry_msgs::Twist stance_transition_velocity_;                    // 站立过渡时的速度命令


    // ==================== LB解锁保护系统成员变量 ====================
    ros::Time lb_unlock_time_;                                          // LB解锁时间
    bool lb_just_unlocked_ = false;                                     // LB是否刚刚解锁
    double lb_unlock_protection_duration_ = 2.0;                        // LB解锁保护时间（秒）

    // ==================== RL步态接收器 ====================
    std::unique_ptr<ocs2::humanoid::RlGaitReceiver> rl_gait_receiver_;

    // // ==================== RL数据存储成员变量 ====================
    // vector_t robot_state_;                                               // 机器人状态
    // SensorData sensor_data_;                                             // 传感器数据
    // vector_t joyCmdState_;                                               // 手柄命令状态
    // CommandData CommandData_;                                            // 命令数据
    
    // // ==================== MPC到RL切换控制 ====================
    // bool enable_rl_switch_ = false;                                      // 是否启用RL切换功能
    // bool mpc_to_rl_switch_requested_ = false;                           // MPC到RL切换请求标志
    // std::mutex rl_switch_mutex_;                                         // RL切换互斥锁
    // std::string rl_config_file_;                                         // RL配置文件路径
    // double dt_ = 0.001;                                                  // 控制周期
    vector6_t torso_interpolation_result_;
    vector_t leg_interpolation_result_;
    vector_t arm_interpolation_result_;

    // ROS 发布者和订阅者
    ros::Subscriber interpolation_request_sub_;
    ros::Subscriber interpolation_status_request_sub_;
    ros::Publisher interpolation_status_pub_;
    ros::Publisher interpolation_result_pub_;

    bool init_fall_down_state_{false};
    // 控制器管理系统：使用控制器管理类统一管理
    std::unique_ptr<RLControllerManager> controller_manager_;  // 控制器管理类
    RLControllerBase* current_controller_ptr_{nullptr};        // 当前控制器指针（从管理类获取）
    
    // 保留 fall_down_state_ 用于向后兼容，但实际逻辑改为控制器切换
    FallStandState fall_down_state_{FallStandState::STANDING}; //是否倒地（已废弃，改为控制器切换）

    bool has_fall_down_controller_{false};
    bool condition_pull_up_mpc_height_{true};
    double switch_distance_threshold_ = 0.003;// MPC-RL切换距离阈值
    double switch_timeout_multiplier_threshold_ = 2.0;// MPC-RL切换超时时间倍率
    double switch_timeout_base_threshold_ = 0.5;// MPC-RL切换基础超时时间（秒）

  };

  class humanoidCheaterController : public humanoidController
  {
  protected:
    void setupStateEstimate(const std::string &taskFile, bool verbose, const std::string &referenceFile) override;
  };

  class humanoidKuavoController : public humanoidController
  {
  protected:
    void setupStateEstimate(const std::string &taskFile, bool verbose, const std::string &referenceFile) override;
  };

} // namespace humanoid_controller
