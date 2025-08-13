#pragma once

#include <humanoid_interface/HumanoidInterface.h>

#include <controller_interface/controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <humanoid_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include "humanoid_interface_ros/visualization/HumanoidVisualizer.h"
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <humanoid_estimation/StateEstimateBase.h>
#include <humanoid_wbc/WbcBase.h>

#include "humanoid_controllers/SafetyChecker.h"
#include "humanoid_controllers/visualization/humanoidSelfCollisionVisualization.h"
#include <sensor_msgs/JointState.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include "kuavo_msgs/gaitTimeName.h"

#include "std_srvs/Trigger.h"

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
#include "humanoid_controllers/shm_manager.h"
#include <std_msgs/Int8.h>
#ifdef USE_DDS
#include "humanoid_controllers/CommonDDS.h"
#endif 

#include "kuavo_common/common/common.h"

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  struct gaitTimeName
  {
    std::string name;
    double startTime;
  };
  struct SensorData
  {
    ros::Time timeStamp_;
    vector_t jointPos_;
    vector_t jointVel_;
    vector_t jointAcc_;
    vector_t jointTorque_;
    vector3_t angularVel_;
    vector3_t linearAccel_;
    Eigen::Quaternion<scalar_t> quat_;
    matrix3_t orientationCovariance_;
    matrix3_t angularVelCovariance_;
    matrix3_t linearAccelCovariance_;
    void resize_joint(size_t num)
    {
      jointPos_.resize(num);
      jointVel_.resize(num);
      jointAcc_.resize(num);
      jointTorque_.resize(num);
    }
  };
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
    void applySensorData();
    void applySensorData(const SensorData &data);
    void updatakinematics(const SensorData &sensor_data, bool is_initialized_);

  protected:
    virtual void updateStateEstimation(const ros::Time &time, bool is_init = false);

    virtual void setupHumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitFile,
                                        bool verbose, RobotVersion rb_version);
    virtual void setupMpc();
    virtual void setupMrt();
    virtual void setupStateEstimate(const std::string &taskFile, bool verbose);
    void sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg);
    void startMpccallback(const std_msgs::Bool::ConstPtr &msg);
    // void checkArmControlModeAndUpdateArmJoint();
    bool armJointSynchronizationCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    
    void robotlocalizationCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    bool enableMmArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    bool getMmArmCtrlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res);
    void real_init_wait();
    void swingArmPlanner(double st, double current_time, double stepDuration, Eigen::VectorXd &desire_arm_q, Eigen::VectorXd &desire_arm_v);
    void headCmdCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg);
    void waistCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void visualizeWrench(const Eigen::VectorXd &wrench, bool is_left);
    bool getCurrentGaitNameCallback(kuavo_msgs::getCurrentGaitName::Request &req, kuavo_msgs::getCurrentGaitName::Response &res);
    void getEnableMpcFlagCallback(const std_msgs::Bool::ConstPtr &msg);
    void getEnableWbcFlagCallback(const std_msgs::Bool::ConstPtr &msg);
    void checkMpcPullUp(double current_time, vector_t & current_state, const TargetTrajectories& planner_target_trajectories);
#ifdef USE_DDS
    void LowStateCallback(const unitree_hg::msg::dds_::LowState_& data);
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

    void dexhandStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

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
    vector_t measuredRbdStateReal_;
    vector_t simplifiedJointPos_;
    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    bool is_initialized_ = false;
    bool is_abnor_StandUp_{false};
    bool wbc_only_{false};
    bool reset_mpc_{false};
    bool disable_mpc_{false};
    bool disable_wbc_{false};
    int hardware_status_ = 0;

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
    bool isPullUp_{false};
    bool setPullUpState_{false};
    double standupTime_{0.0};
    std::shared_ptr<WbcBase> standUpWbc_;
    vector_t curRobotLegState_;

    // Visualization
    std::shared_ptr<HumanoidVisualizer> robotVisualizer_;
    // std::shared_ptr<biped_robot::BipedRobotVisualizer> bipedRobotVisualizer_;//TODO: check if this is needed
    ros::Publisher observationPublisher_;
    ros::Publisher wbc_observation_publisher_;
    // Controller Interface
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
    ros::Subscriber waist_sub_;
    ros::Subscriber head_array_sub_;
    ros::Subscriber arm_joint_traj_sub_;
    ros::Subscriber mm_arm_joint_traj_sub_;
    ros::Subscriber arm_target_traj_sub_;//最终的手臂目标位置
    ros::Subscriber foot_pos_des_sub_;
    ros::Subscriber hand_wrench_sub_;
    ros::Subscriber arm_control_mode_sub_;
    ros::Subscriber contact_force_sub_;
    ros::Publisher mpcPolicyPublisher_;

    ros::Subscriber dexhand_state_sub_;

    ros::ServiceServer armJointSynchronizationSrv_;
    ros::Subscriber enable_mpc_sub_;
    ros::Subscriber enable_wbc_sub_;

    ros::ServiceServer enableArmCtrlSrv_;
    ros::ServiceServer enableMmArmCtrlSrv_;
    ros::ServiceServer getMmArmCtrlSrv_;
    ros::ServiceServer currentGaitNameSrv_;
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
#ifdef KUAVO_CONTROL_LIB_FOUND
    HighlyDynamic::JointFilter *joint_filter_ptr_{nullptr};
#endif
    HighlyDynamic::KuavoSettings kuavo_settings_;
    bool is_nodelet_node_{false};

    void publishFeetTrajectory(const TargetTrajectories &targetTrajectories);

    ros::ServiceServer real_initial_start_service_;
    KuavoDataBuffer<SensorData> *sensors_data_buffer_ptr_;
    bool is_real_{false};
    bool is_cali_{false};
    char initial_input_cmd_ = '\0';
    // TrajectoryPublisher *trajectory_publisher_{nullptr};
    double dt_ = 0.001;
    std::thread mpcThread_;
    std::atomic_bool controllerRunning_{}, mpcRunning_{};
    benchmark::RepeatedTimer mpcTimer_;
    benchmark::RepeatedTimer wbcTimer_;
    size_t jointNum_ = 12;
    size_t armNum_ = 0;
    size_t headNum_ = 2;
    size_t waistNum_ = 1;
    size_t jointNumReal_ = 12;
    size_t armNumReal_ = 0;
    size_t actuatedDofNumReal_ = 12;// 实物的自由度
    ArmControlMode mpcArmControlMode_ = ArmControlMode::AUTO_SWING; // KEEP = 0, AUTO_SWING = 1, EXTERN_CONTROL = 2
    ArmControlMode mpcArmControlMode_desired_ = ArmControlMode::AUTO_SWING; // KEEP = 0, AUTO_SWING = 1, EXTERN_CONTROL = 2
    int armDofMPC_ = 7; // 单手臂的自由度，会从配置文件中重新计算
    int armDofReal_ = 7; // 实际单手臂的自由度
    int armDofDiff_ = 0; // 单手臂的自由度差

    bool is_simplified_model_ = false;// 是否是简化的MPC模型
    TargetTrajectories currentArmTargetTrajectories_;// 当前手臂的目标轨迹，简化模型的关节target将会从这里读取
    int seq_ = 0;
    SensorData sensor_data_head_;
    SensorData sensor_data_waist_;
    Eigen::Quaterniond robot_quat_state_update_;
    vector_t desire_head_pos_ = vector_t::Zero(2);
    vector_t desire_waist_pos_ = vector_t::Zero(1);  // 腰部目标位置
    vector_t desire_arm_q_prev_;
    vector_t jointPos_, jointVel_;
    vector_t jointAcc_;
    vector_t jointTorque_;
    vector_t dexhand_joint_pos_ = vector_t::Zero(12);

    vector_t jointPosWBC_, jointVelWBC_;
    vector_t jointAccWBC_;
    vector_t jointCurrentWBC_;

    vector_t motor_c2t_;
    std::vector<std::vector<double>> motor_cul;
    std::vector<std::vector<double>> motor_coeff;
    bool init_input_ = false;
    Eigen::Quaternion<scalar_t> quat_;
    Eigen::Quaternion<scalar_t> quat_init;
    contact_flag_t contactFlag_;
    vector3_t angularVel_, linearAccel_;
    matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
    size_t plannedMode_ = ModeNumber::SS;
    size_t estPlannedMode_ = ModeNumber::SS;
    size_t nextMode_ = ModeNumber::SS;
    vector_t defalutJointPos_;
    vector_t initial_status_;
    vector_t intail_input_;
    vector_t joint_kp_, joint_kd_, joint_kp_walking_, joint_kd_walking_, head_kp_, head_kd_, waist_kp_, waist_kd_;  // 添加腰部PD控制增益
    vector_t pull_up_status_;
    vector_t pull_up_input_;
    vector_t cur_status_;
    vector_t cur_input_;
    vector_t output_tau_, output_pos_, output_vel_;
    Eigen::MatrixXd joint_state_limit_; // 26x2, lower and upper limit
    double contact_cst_st_ = 0.1;
    double contact_cst_et_ = 0.1;
    double robotMass_ = 50;

    const std::string robotName_ = "humanoid";
    bool use_external_mpc_{true};
    bool use_joint_filter_{false};
    bool use_estimator_contact_{false};
    bool is_stance_mode_{false};
    bool only_half_up_body_{false};
    bool wheel_arm_robot_{false};
    bool stand_up_protect_{false};
    
    TopicLogger *ros_logger_{nullptr};
    vector_t optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_;
    size_t optimized_mode_;
    bool is_play_back_mode_ = false;
    int control_mode_ = 2; // 0：CST, 1: CSV, 2:CSP
    bool reset_estimator_ = false;

    LowPassFilter2ndOrder acc_filter_;
    // LowPassFilter2ndOrder free_acc_filter_;
    LowPassFilter2ndOrder gyro_filter_;
    LowPassFilter2ndOrder arm_joint_pos_filter_;
    LowPassFilter2ndOrder arm_joint_vel_filter_;
    LowPassFilter2ndOrder mrt_joint_vel_filter_;


    bool use_ros_arm_joint_trajectory_ = false;
    bool use_mm_arm_joint_trajectory_ = false;
    ArmJointTrajectory arm_joint_trajectory_;
    ArmJointTrajectory mm_arm_joint_trajectory_;
    vector_t arm_joint_pos_cmd_prev_;
    vector_t joint_control_modes_;
    std::map<std::string, ModeSequenceTemplate> gait_map_;

    bool is_swing_arm_ = false;
    double swing_arm_gain_{0.0}, swing_elbow_scale_{0.0};
    double ruiwo_motor_velocities_factor_{0.0};
    gaitTimeName current_gait_{"stance", 0.0}, last_gait_{"stance", 0.0};
    vector_t desire_arm_q, desire_arm_v;

    // 共享内存通讯
    std::unique_ptr<gazebo_shm::ShmManager> shm_manager_;
    bool use_shm_communication_{false};  // 是否使用共享内存通讯
    bool updateSensorDataFromShm();      // 从共享内存更新传感器数据
    void publishJointCmdToShm(const kuavo_msgs::jointCmd& jointCmdMsg);         // 发布关节命令到共享内存
    
    // CPU内核隔离设置
    void setupCpuIsolation();  // 从ROS参数获取隔离CPU索引并设置线程亲和性
    
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
    std::unique_ptr<HumanoidControllerDDSClient> dds_client_;
#endif
    
    // Latest sensor data for comparison
    SensorData latest_dds_sensor_data_;
    SensorData latest_ros_sensor_data_;
    bool has_dds_data_ = false;
    bool has_ros_data_ = false;
    std::vector<std::pair<double, double> > waist_joint_limits_ = {{-120, 120}};  // 添加腰部关节限制

    void publishWbcArmEndEffectorPose();

  };

  class humanoidCheaterController : public humanoidController
  {
  protected:
    void setupStateEstimate(const std::string &taskFile, bool verbose) override;
  };

  class humanoidKuavoController : public humanoidController
  {
  protected:
    void setupStateEstimate(const std::string &taskFile, bool verbose) override;
  };

} // namespace humanoid_controller
