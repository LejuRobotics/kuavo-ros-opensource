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

  bool checkSitDownComplete(scalar_t initTime, const vector_t& initState);
  bool isSeatLegActionEnabled() const;
  void onSeatReturnPreUpdate(const std_msgs::Int8::ConstPtr& msg);
  void resetSeatLifecycleToIdle();

  ros::NodeHandle& nh_;
  ros::ServiceServer sitDownService_;
  ros::Publisher sitDownCompletePub_;
  ros::ServiceServer standUpFromSitService_;
  ros::Publisher stopRobotPub_;
  ros::Publisher seatReturnPreUpdatePub_;
  ros::Subscriber sub_seat_return_preupdate_done_;

  SeatState seatState_ = SeatState::IDLE;

  vector_t defaultSitDownTargetState_;
  double sitDownDurationSeconds_{2.0};
  scalar_t sitDownStartTime_ = 0.0;
  vector_t sitDownStartState_;
  vector_t sitDownTargetState_;

  RobotVersion rbVersion_;
  bool isFirstUpdate_ = true;
};

}  // namespace humanoid
}  // namespace ocs2
