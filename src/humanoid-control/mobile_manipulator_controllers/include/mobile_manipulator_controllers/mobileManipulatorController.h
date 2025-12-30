#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorSpatialKinematics.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <std_srvs/SetBool.h> 
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
// #include <humanoid_interface/HumanoidInterface.h>
#include <mobile_manipulator_controllers/mobileManipulatorVisualization.h>
#include "kuavo_msgs/changeTorsoCtrlMode.h"
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <yaml-cpp/yaml.h>
#include "mobile_manipulator_controllers/package_path.h"


namespace mobile_manipulator_controller
{
  using namespace ocs2;
  // using namespace humanoid;
  using namespace ocs2::mobile_manipulator;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  
  enum class MpcType
  {
    DDP,
    SQP
  };

  enum class ControlType
  {
    None = 0,
    ArmOnly,
    BaseOnly,
    BaseArm, // 通过base_pose_cmd强制控制base位置
  };

  std::string controlTypeToString(ControlType controlType);

  class MobileManipulatorController
  {
  public:
    MobileManipulatorController() = default;
    ~MobileManipulatorController();
    bool init(ros::NodeHandle &nh, int freq);
    void update();
    void stop() { mpcRunning_ = false; newMPCSolved_ = false;}
    bool recievedObservation() const { return recievedObservation_; }
    
    // 重置MPC和MRT的方法
    void resetMpc();
    void resetMrt();
    void resetMpcMrt();
    void restart() {mpcRunning_ = true;}

  protected:
    void starting();
    virtual void setupMobileManipulatorInterface(const std::string &taskFile, const std::string &libFolder, const std::string &urdfFile, MpcType mpcType);
    virtual void setupMpc();
    virtual void setupMrt();
    virtual void convertObservationfromHumanoid2MM(const SystemObservation& humanoidObservation, SystemObservation& mmOservation);
    virtual void convertObservationfromMM2Humanoid(const SystemObservation& mmObservation, const SystemObservation& currentHumanoidObservation, SystemObservation& humanoidObservation);
    std::pair<vector_t, vector_t> convertStateInputfromMM2Humanoid(const vector_t& mmState, const vector_t& mmInput, const SystemObservation& currentHumanoidObservation);
    virtual void controlHumanoid(const vector_t& mmState, const vector_t& mmInput, const SystemObservation& currentHumanoidObservation);

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

    ros::Time last_time_;
    ros::Time current_time_;

    // std::thread keyboardThread_;
    std::thread mpcThread_;

    std::atomic_bool controllerRunning_{}, mpcRunning_{}, newMPCSolved_{false};
    benchmark::RepeatedTimer mpcTimer_;

    // Interface
    // std::shared_ptr<HumanoidInterface> humanoidInterface_;
    std::shared_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> mobileManipulatorInterface_;
    std::shared_ptr<PinocchioInterface> pinocchioInterfaceWBCPtr_;
    // std::shared_ptr<HumanoidInterface> HumanoidInterface_mpc;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsWBCPtr_;
    // std::shared_ptr<PinocchioEndEffectorSpatialKinematics> eeSpatialKinematicsPtr_;

    SystemObservation humanoidObservation_, mmObservation_;
    SystemObservation mmObservationDummy_;
    vector_t simplifiedJointPos_;
    bool is_initialized_ = false;
    bool reset_mpc_{false};

    // Nonlinear MPC
    std::shared_ptr<MPC_BASE> mpc_;
    std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;
    // std::shared_ptr<MRT_ROS_Interface> mrtRosInterface_;

    ros::Publisher observationPublisher_;
    ros::Publisher mpcPolicyPublisher_;

    ros::Publisher stop_pub_;
    ros::Subscriber humanoidObservationSub_;
    ros::Subscriber terrainHeightSubscriber_;
    ros::Subscriber basePoseCmdSubscriber_;

    std::unique_ptr<PinocchioInterface> pinocchioInterface_ptr_;
    std::shared_ptr<PinocchioEndEffectorSpatialKinematics> eeSpatialKinematicsPtr_;
    std::unique_ptr<MobileManipulatorPinocchioMapping> pinocchioMappingPtr_;
    // CentroidalModelInfo centroidalModelInfo_;
    // CentroidalModelInfo centroidalModelInfoWBC_;

    // Node Handle
    ros::NodeHandle controllerNh_;
    const std::string robotName_ = "mobile_manipulator";
    MpcType mpcType_;
    bool recievedObservation_ = false;

  private:
    void humanoidObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);
    void limitHumanoidTargetState(vector_t& humanoidTargetState);
    TargetTrajectories generateTargetTrajectories(const vector_t& currentState, const vector_t& desiredState, const SystemObservation& currentHumanoidObservation);
    void controlBase(const vector_t& mmState, const vector_t& mmInput);
    void controlBasePos(const vector_t& mmState, const vector_t& mmInput);
    SystemObservation forwardSimulation(const SystemObservation& currentObservation);
    bool controlService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
    bool getKinematicMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
    bool resetMpcService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
    bool resetMpcMrtService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool toggleMpcService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void pubHumanoid2MMTf();
    vector_t getMMEefPose(const vector_t& state);
    visualization_msgs::MarkerArray getVisualizeTrajectoryMsg(const std::deque<Eigen::VectorXd>& twoHandPoseTrajectory, std::vector<double> rgba={1,0,0,1});

    bool limitArmPosition(ocs2::vector_t& armPosition);
    void targetCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg);

    ocs2::mobile_manipulator::ManipulatorModelInfo info_;
    std::shared_ptr<MobileManipulatorVisualization> visualizationPtr_;

    ros::Publisher humanoidTargetTrajectoriesPublisher_;
    ros::Publisher humanoidTorsoTargetTrajectoriesPublisher_;
    ros::Publisher humanoidArmTargetTrajectoriesPublisher_;
    ros::Publisher humanoidCmdVelPublisher_;
    ros::Publisher humanoidCmdPosPublisher_;
    ros::Publisher mmEefPosesPublisher_;
    ros::Publisher mmPlanedTrajPublisher_;
    ros::Publisher armTrajPublisher_;
    ros::ServiceServer kinematicMpcControlSrv_;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster_;
    ros::ServiceServer getKinematicMpcControlModeSrv_;
    ros::ServiceServer resetMpcSrv_;
    ros::ServiceServer resetMpcMrtSrv_;
    ros::ServiceServer toggleMpcSrv_;
    ros::Subscriber targetSubscriber_;

    double comHeight_;
    size_t humanoidStateDim_{38};//12+12+14
    size_t humanoidInputDim_{62};//3*8+2*6+12+14
    int freq_;
    int dummySim_{1};
    int dummySimArm_{1};
    ControlType controlType_ = ControlType::None;
    bool velControl_{true};
    Vector6d basePoseDeltaLimit_;
    // Vector6d basePoseCmd_;
    ros::Time basePoseCmdUpdatedTime_;
    double terrain_height_{0};
    bool visualizeMm_{true};
    std::deque<Eigen::VectorXd> mmPlanedTrajQueue_;
    YAML::Node yaml_cfg_;
    std::vector<double> arm_min_;
    std::vector<double> arm_max_;
    
    // 重置相关的互斥锁和标志
    std::mutex resetMutex_;
    std::atomic<bool> isResetting_{false};

    // anp
    std::atomic<bool> newTargetReceived_{false};
    ros::Time lastTargetTime_;
    std::mutex targetMutex_;
  };

} // namespace mobile_manipulator_controller
