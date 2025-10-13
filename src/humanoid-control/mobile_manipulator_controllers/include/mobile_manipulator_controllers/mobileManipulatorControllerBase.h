#pragma once

#include <pinocchio/fwd.hpp>
#include <ros/ros.h>
#include "mobile_manipulator_controllers/mobileManipulatorIkTarget.h"
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorSpatialKinematics.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <mobile_manipulator_controllers/mobileManipulatorVisualization.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <memory>
#include "mobile_manipulator_controllers/package_path.h"
#include "mobile_manipulator_controllers/TopicLogger.h"
#include "kuavo_msgs/MmDetectionMsg.h"

namespace mobile_manipulator_controller
{
  using namespace ocs2;
  using namespace ocs2::mobile_manipulator;
  
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

  class MobileManipulatorControllerBase
  {
  public:
    MobileManipulatorControllerBase(ros::NodeHandle &nh, const std::string& taskFile, const std::string& libFolder, const std::string& urdfFile, MpcType mpcType, int freq, 
      ControlType control_type=ControlType::BaseArm, bool dummySimArm=true, bool visualizeMm=true);
    ~MobileManipulatorControllerBase();
    int update(const vector_t& externalState, vector_t& nextState);
    int update(const vector_t& externalState, vector_t& nextState, vector_t& optimizedInput);
    void stop() { updateRunning_ = false; observationPublishing_ = false; ikTargetManager_->stop(); mpcRunning_ = false; }
    void start() { updateRunning_ = true; mpcRunning_ = true; }
    bool isObservationPublishing() const { return observationPublishing_; }
    /**
     * @brief Reset the controller to the given external state
     * @param externalState: The external state to reset to
     * @return 0 if successful, -1 if failed
     */
    int reset(const vector_t& externalState);
    inline void setExternalState(const vector_t& externalState) {
      vector_t nextState = vector_t::Zero(externalState.size());
      this->update(externalState, nextState);
      mmObservationDummy_.time += 1.0 / freq_;
    }
    int setTargetTrajectory(const TargetTrajectories& targetTrajectories);
    ocs2::vector_t getTargetFromState(const vector_t& state);
    void resetMpcToGivenState(const SystemObservation& resetObservation);
    void starting();
    virtual void setupMobileManipulatorInterface(const std::string &taskFile, const std::string &libFolder, const std::string &urdfFile, MpcType mpcType);
    virtual void setupMpc();
    virtual void setupMrt();
    void setAnomalyStopMpc(bool anomalyStopMpc){anomalyStopMpc_ = anomalyStopMpc;};
    void setAnomalyCheckThreshold(double positionErrorMaxThreshold, double orientationErrorMaxThreshold, double positionErrorAvgThreshold, double orientationErrorAvgThreshold, double VelocityErrorAvgThreshold, 
      double VelocityErrorMaxThreshold)
      {
        positionErrorMaxThreshold_ = positionErrorMaxThreshold;
        orientationErrorMaxThreshold_ = orientationErrorMaxThreshold;
        positionErrorAvgThreshold_ = positionErrorAvgThreshold;
        orientationErrorAvgThreshold_ = orientationErrorAvgThreshold;
        VelocityErrorAvgThreshold_ = VelocityErrorAvgThreshold;
        VelocityErrorMaxThreshold_ = VelocityErrorMaxThreshold;
      };
    int getOptimizedStateAndInput(const SystemObservation& currentObservation, vector_t& optimizedStateMrt, vector_t& optimizedInputMrt);
    int anomaly_check(const SystemObservation& currentObservation, const PrimalSolution& policy, const CommandData& command, const PerformanceIndex& performanceIndices);
    /**
     * @brief 检查MPC线程是否已经完全暂停
     * @return true如果线程已暂停，false如果线程仍在运行
     */
    bool isMpcThreadPaused() const;        /**
     * Creates MPC Policy message.
     *
     * @param [in] primalSolution: The policy data of the MPC.
     * @param [in] commandData: The command data of the MPC.
     * @param [in] performanceIndices: The performance indices data of the solver.
     * @return MPC policy message.
     */
    static ocs2_msgs::mpc_flattened_controller createMpcPolicyMsg(const PrimalSolution &primalSolution, const CommandData &commandData,
                                                                  const PerformanceIndex &performanceIndices);
    SystemObservation forwardSimulation(const SystemObservation& currentObservation, scalar_t dt);
    vector_t getMMEefPose(const vector_t& state);
    visualization_msgs::MarkerArray getVisualizeTrajectoryMsg(const std::deque<Eigen::VectorXd>& twoHandPoseTrajectory, std::vector<double> rgba={1,0,0,1});

    std::shared_ptr<MobileManipulatorVisualization> visualizationPtr_;

    ros::Publisher mmEefPosesPublisher_;
    ros::Publisher mmPlanedTrajPublisher_;
    void publishEefPoses(const vector_t& eefPoses);

    int freq_;
    int dummySim_;
    int dummySimArm_;
    bool visualizeMm_;
    bool anomalyStopMpc_{false};
    std::deque<Eigen::VectorXd> mmPlanedTrajQueue_;

  private:

    double VelocityErrorAvgThreshold_{0.5};
    double VelocityErrorMaxThreshold_{2.0};
    double positionErrorMaxThreshold_{0.15};
    double orientationErrorMaxThreshold_{0.4};
    double positionErrorAvgThreshold_{0.15};
    double orientationErrorAvgThreshold_{0.4};

    double errorAnglerEX_{0.0};

    // 末端效应器位置姿态误差结构
    struct EefPoseError {
      vector_t positionErrors;      // 位置误差 (双手 x 3维)
      vector_t orientationErrors;   // 姿态误差 (双手角度误差)  
      vector_t totalErrors;         // 总误差 (所有14维)
    };

    // 异常检测结构
    struct DetectionResult {
      bool hasAnomaly{false};
      std::string anomalyType;
      
      // 第一类检测：轨迹级别的位置误差
      double trajectoryPositionErrorMax{0.0};
      double trajectoryPositionErrorAvg{0.0};
      bool trajectoryPositionMaxThresholdExceeded{false};
      bool trajectoryPositionAvgThresholdExceeded{false};
      
      // 第一类检测：轨迹级别的姿态误差
      double trajectoryOrientationErrorMax{0.0};
      double trajectoryOrientationErrorAvg{0.0};
      bool trajectoryOrientationMaxThresholdExceeded{false};
      bool trajectoryOrientationAvgThresholdExceeded{false};
      
      // 第二类检测：速度阈值检测
      double velocityMagnitudeAvg{0.0};
      double velocityMagnitudeMax{0.0};
      bool velocityAvgThresholdExceeded{false};
      bool velocityMaxThresholdExceeded{false};
    };
  

    vector_t getStateFromPolicyTrajectory(const scalar_array_t& timeTrajectory, const vector_array_t& stateTrajectory, scalar_t currentTime);
    std::pair<std::vector<vector_t>, std::vector<scalar_t>> calculatePolicyEefVelocities(const PrimalSolution& policy);
    
    // Helper functions for velocity calculation
    vector_t calculateSingleEefVelocity(const vector_t& eefPose1, const vector_t& eefPose2, scalar_t dt);
    vector_t calculateDualHandEefVelocity(const vector_t& eefPoses1, const vector_t& eefPoses2, scalar_t dt);
    std::vector<vector_t> calculateTargetEefVelocities(const TargetTrajectories& targetTrajectories, const std::vector<scalar_t>& times);
    EefPoseError calculateEefPoseErrors(const vector_t& optimizedEefPoses, const vector_t& targetEefPoses);
    std::vector<vector_t> calculateEefVelocityErrors(const std::vector<vector_t>& policyVel, const std::vector<vector_t>& targetVel);
    DetectionResult detectEefAnomalies(const EefPoseError& poseError, const std::vector<vector_t>& velErrors);
    DetectionResult detectEefAnomaliesNew(const std::vector<EefPoseError>& poseErrors, const std::vector<scalar_t>& velocityMagnitudes);
    void publishDetectionResult(const DetectionResult& result);
  protected:
    ControlType controlType_ = ControlType::None;
    ControlType lastControlType_ = ControlType::ArmOnly; // 初始化为ArmOnly，在第一次启动时，可以打印出MPC的初始状态
    std::mutex control_type_mutex_;
    std::thread mpcThread_;

    std::atomic_bool controllerRunning_{}, mpcRunning_{}, updateRunning_{};
    
    // Thread synchronization for reset operations
    mutable std::mutex mpcThreadMutex_;
    std::condition_variable mpcThreadCondition_;
    std::atomic_bool mpcThreadPaused_{false};
    
    // Flag to track if observations are being published
    std::atomic_bool observationPublishing_{false};
    
    // Thread synchronization configuration
    int maxThreadWaitTimeMs_{1000};  // 最大线程等待时间(毫秒)
    int threadCheckIntervalMs_{2};   // 线程检查间隔(毫秒)
    
    benchmark::RepeatedTimer mpcTimer_;

    // Interface
    std::shared_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> mobileManipulatorInterface_;
    ocs2::mobile_manipulator::ManipulatorModelInfo info_;

    SystemObservation mmObservationDummy_;

    // Nonlinear MPC
    std::shared_ptr<MPC_BASE> mpc_;
    std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

    ros::Publisher observationPublisher_;
    ros::Publisher mpcPolicyPublisher_;
    ros::Publisher mmDetectionResultPublisher_;
    std::unique_ptr<PinocchioInterface> pinocchioInterface_ptr_;
    std::shared_ptr<PinocchioEndEffectorSpatialKinematics> eeSpatialKinematicsPtr_;
    std::unique_ptr<MobileManipulatorPinocchioMapping> pinocchioMappingPtr_;

    // Node Handle
    ros::NodeHandle controllerNh_;
    const std::string robotName_ = "mobile_manipulator";
    std::unique_ptr<humanoid::TopicLogger> ros_logger_;
    MpcType mpcType_;
    unsigned int updateCount_{0};

    std::unique_ptr<MobileManipulatorIkTarget> ikTargetManager_;
  };

} // namespace mobile_manipulator_controller
