#pragma once

#include <pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <std_srvs/SetBool.h> 
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <mobile_manipulator_controllers/mobileManipulatorControllerBase.h>
#include "kuavo_msgs/changeTorsoCtrlMode.h"
#include <yaml-cpp/yaml.h>
#include "mobile_manipulator_controllers/package_path.h"


namespace mobile_manipulator_controller
{
  using namespace ocs2;
  using namespace ocs2::mobile_manipulator;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  class MobileManipulatorController : public MobileManipulatorControllerBase
  {
  public:
    MobileManipulatorController(ros::NodeHandle &nh, const std::string& taskFile, const std::string& libFolder, const std::string& urdfFile, MpcType mpcType, int freq, 
      ControlType control_type=ControlType::BaseArm, bool dummySimArm=true, bool visualizeMm=true, bool anomalyStopMpc=false);
    ~MobileManipulatorController();
    bool init(ros::NodeHandle &nh);
    void update();
    bool recievedObservation() const { return recievedObservation_; }

  protected:
    virtual void convertObservationfromHumanoid2MM(const SystemObservation& humanoidObservation, SystemObservation& mmOservation);
    virtual void convertObservationfromMM2Humanoid(const SystemObservation& mmObservation, const SystemObservation& currentHumanoidObservation, SystemObservation& humanoidObservation);
    std::pair<vector_t, vector_t> convertStateInputfromMM2Humanoid(const vector_t& mmState, const vector_t& mmInput, const SystemObservation& currentHumanoidObservation);
    virtual void controlHumanoid(const vector_t& mmState, const vector_t& mmInput, const SystemObservation& currentHumanoidObservation);

  private:
    void humanoidObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);
    void limitHumanoidTargetState(vector_t& humanoidTargetState);
    TargetTrajectories generateTargetTrajectories(const vector_t& currentState, const vector_t& desiredState, const SystemObservation& currentHumanoidObservation);
    void controlBasePos(const vector_t& mmState, const vector_t& mmInput);
    bool controlService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
    bool getKinematicMpcControlModeService(kuavo_msgs::changeTorsoCtrlMode::Request& req, kuavo_msgs::changeTorsoCtrlMode::Response& res);
    void pubHumanoid2MMTf();
    bool limitArmPosition(ocs2::vector_t& armPosition);
    // for play back mode
    void mmStateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void mmControlTypeCallback(const std_msgs::Int8::ConstPtr& msg);

    // ROS Communication Layer
    ros::Subscriber humanoidObservationSub_;
    ros::Subscriber terrainHeightSubscriber_;
    ros::Publisher humanoidTargetTrajectoriesPublisher_;
    ros::Publisher humanoidTorsoTargetTrajectoriesPublisher_;
    ros::Publisher humanoidArmTargetTrajectoriesPublisher_;
    ros::Publisher humanoidCmdVelPublisher_;
    ros::Publisher humanoidCmdPosPublisher_;
    ros::Publisher armTrajPublisher_;
    ros::Publisher mmStatePublisher_;
    ros::Publisher mmControlTypePublisher_;
    ros::ServiceServer kinematicMpcControlSrv_;
    ros::ServiceServer getKinematicMpcControlModeSrv_;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster_;

    // Humanoid specific state management
    SystemObservation humanoidObservation_, mmObservation_;
    double comHeight_;
    double terrain_height_{0};
    size_t humanoidStateDim_{38};//12+12+14
    size_t humanoidInputDim_{62};//3*8+2*6+12+14
    bool recievedObservation_ = false;
    
    // Control configuration
    // ControlType controlType_ = ControlType::None;
    // ControlType lastControlType_ = ControlType::ArmOnly; // 初始化为ArmOnly，在第一次启动时，可以打印出MPC的初始状态

    bool mpcInitialized_ = false;

    Vector6d basePoseDeltaLimit_;
    ros::Time basePoseCmdUpdatedTime_;
    
    // Configuration from YAML
    YAML::Node yaml_cfg_;
    std::vector<double> arm_min_;
    std::vector<double> arm_max_;
    // play back mode
    bool is_play_back_mode_ = false;
    ros::Subscriber mmControlTypeSubscriber_;
    uint32_t observation_count_ = 0;
  };

} // namespace mobile_manipulator_controller
