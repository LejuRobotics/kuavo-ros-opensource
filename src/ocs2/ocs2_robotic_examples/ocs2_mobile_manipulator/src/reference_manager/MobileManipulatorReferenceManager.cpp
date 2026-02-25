#include "ocs2_mobile_manipulator/reference_manager/MobileManipulatorReferenceManager.h"
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {
namespace mobile_manipulator {
  MobileManipulatorReferenceManager::MobileManipulatorReferenceManager(const ManipulatorModelInfo& info)
  : ReferenceManager(TargetTrajectories(), ModeSchedule())
  , info_(info)
  {
    baseStateDim_ = info_.stateDim - info_.armDim;
    Q_ = matrix_t::Zero(info_.stateDim, info_.stateDim);
    // std::cout << "Q matrix is: \n" << Q_ << std::endl;
    R_ = matrix_t::Zero(info_.inputDim, info_.inputDim);

    nodeHandle_ = ros::NodeHandle("~");
    subQMatrix_ = nodeHandle_.subscribe("/mobile_manipulator_q_matrix_diagonal", 1, &MobileManipulatorReferenceManager::callbackQMatrix, this);
    subRMatrix_ = nodeHandle_.subscribe("/mobile_manipulator_r_matrix_diagonal", 1, &MobileManipulatorReferenceManager::callbackRMatrix, this);

    subObservation_ = nodeHandle_.subscribe("/mobile_manipulator_mpc_observation", 1, &MobileManipulatorReferenceManager::callbackObservation, this);
  }

  void MobileManipulatorReferenceManager::callbackQMatrix(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(msg->data.size() != baseStateDim_)
    {
      ROS_ERROR_STREAM("The size of the Q matrix diagonal is not equal to the base state dimension.");
      return;
    }
    for(size_t i = 0; i < baseStateDim_; i++)
    {
      Q_(i, i) = msg->data[i];
    }
    std::cout << "Q matrix is updated." << std::endl;
    std::cout << "Q matrix diagonal is: " << Q_.diagonal().transpose() << std::endl;
    QTime_ = latestObservation_.time;
    updatedQ_ = true;
  }

  void MobileManipulatorReferenceManager::callbackRMatrix(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(msg->data.size() != info_.inputDim)
    {
      ROS_ERROR_STREAM("The size of the R matrix diagonal is not equal to the input dimension.");
      return;
    }
    for(size_t i = 0; i < info_.inputDim; i++)
    {
      R_(i, i) = msg->data[i];
    }
    std::cout << "R matrix is updated." << std::endl;
    std::cout << "R matrix diagonal is: " << R_.diagonal().transpose() << std::endl;
    RTime_ = latestObservation_.time;
    updatedR_ = true;
  }

  void MobileManipulatorReferenceManager::callbackObservation(const ocs2_msgs::mpc_observation::ConstPtr &msg)
  {
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  }

  void MobileManipulatorReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule){
    // std::cout << "MobileManipulatorReferenceManager::modifyReferences() is not implemented yet." << std::endl;
  }
}  // namespace mobile_manipulator
}  // namespace ocs2