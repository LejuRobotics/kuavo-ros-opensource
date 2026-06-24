#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <humanoid_interface/gait/GaitSchedule.h>
#include <kuavo_common/common/common.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>

namespace ocs2 {
namespace humanoid {

enum class SeatState {
  IDLE,
  SITTING_DOWN,
  SIT_DOWN_COMPLETE,
  STANDING_UP,
};

class SitReferenceManager {
 public:
  SitReferenceManager(ros::NodeHandle& nh,
                      RobotVersion rbVersion);

  ~SitReferenceManager() = default;

  void setupServices(const std::string& nodeHandleName);

  bool update(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
              TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule,
              const CentroidalModelInfo& info);

  bool isSitDownOrStandUpActive() const {
    return seatState_ == SeatState::SITTING_DOWN || seatState_ == SeatState::STANDING_UP;
  }

 private:
  bool sitDownSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool standUpFromSitSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  TargetTrajectories generateTargetForSitDown(scalar_t initTime, scalar_t finalTime,
                                              const vector_t& initState, const CentroidalModelInfo& info);
  TargetTrajectories generateTargetForStandUpFromSit(scalar_t initTime, scalar_t finalTime,
                                                     const vector_t& initState, const CentroidalModelInfo& info);

  bool checkSitDownComplete(scalar_t initTime, const vector_t& initState);
  bool checkStandUpFromSitComplete(scalar_t initTime, const vector_t& initState);
  void finishStandUpTransition(scalar_t reportHeight, double reportTarget);
  void buildP1StandUpLerpEnd(const vector_t& initState, int stateDim);

  ros::NodeHandle& nh_;
  ros::ServiceServer sitDownService_;
  ros::Publisher sitDownCompletePub_;
  ros::Publisher sitDownSecondPhaseWbcPub_;
  ros::Publisher sitDownThirdPhaseLegPub_;
  ros::ServiceServer standUpFromSitService_;
  ros::Publisher standUpFromSitCompletePub_;
  ros::Publisher sitUnfreezePub_;
  ros::Publisher stopRobotPub_;

  SeatState seatState_ = SeatState::IDLE;
  /** 1: P2→P1 (sitDownStart); 2: P3→P2 (sitDownTarget) */
  int standUpLerpTarget_{1};

  vector_t defaultSitDownTargetState_;
  double sitDownDurationSeconds_{2.0};
  scalar_t sitDownStartTime_ = 0.0;
  vector_t sitDownStartState_;
  vector_t sitDownTargetState_;

  scalar_t standUpFromSitStartTime_ = 0.0;
  vector_t standUpFromSitLerpStart_;
  vector_t standUpFromSitLerpEnd_;

  RobotVersion rbVersion_;
  bool isFirstUpdate_ = true;
};

}  // namespace humanoid
}  // namespace ocs2
