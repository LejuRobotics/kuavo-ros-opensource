
#ifndef SRC_QMTARGETTRAJECTORIESPUBLISHER_H
#define SRC_QMTARGETTRAJECTORIESPUBLISHER_H

#include <functional>
#include <memory>
#include <mutex>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace ocs2{
namespace humanoid {

/**
 * This class lets the user to command robot form interactive marker.
 */
class HandTargetTrajectoriesInteractiveMarker final {
 public:
  using GoalPoseToTargetTrajectories = std::function<TargetTrajectories(
      const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, 
      const SystemObservation& observation, const vector_t& lastEeState)>;

  /**
   * Constructor
   *
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] boxIndex:  Index set to distinguish different boxes
   * @param [in] TopicPrefix: the latest observation is be expected on "subTopicPrefix_mpc_observation" topic.
   * @param [in] gaolPoseToTargetTrajectories: A function which transforms the commanded pose to TargetTrajectories.
   */
  HandTargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nodeHandle, uint8_t boxIndex, 
                                      const std::string& TopicPrefix, GoalPoseToTargetTrajectories goalPoseToTargetTrajectories);

  /**
   * Spins ROS to update the interactive markers.
   */
  void publishInteractiveMarker() { ::ros::spin(); }

 private:
  visualization_msgs::InteractiveMarker createInteractiveMarker() const;
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;

  GoalPoseToTargetTrajectories goalPoseToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

  ::ros::Subscriber observationSubscriber_, eePoseSub_;
  mutable std::mutex latestObservationMutex_, latestObservationEeMutex_;
  SystemObservation latestObservation_;

  uint8_t boxIndex_;

  static vector_t lastEeState_;
  ros::Publisher ArmTargetTrajectoriesPublisher_;

};

}
}




#endif //SRC_QMTARGETTRAJECTORIESPUBLISHER_H
