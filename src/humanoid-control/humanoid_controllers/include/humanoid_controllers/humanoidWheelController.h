#pragma once

#include <ros/ros.h>
#include "humanoid_interface/common/Types.h"
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/sensorsData.h"
#include <nav_msgs/Odometry.h>
#include "kuavo_common/common/sensor_data.h"
#include "humanoid_interface_drake/kuavo_data_buffer.h"
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_core/misc/LoadData.h>

#include "humanoid_wheel_interface/HumanoidWheelInterface.h"
#include "humanoid_interface/common/TopicLogger.h"
#include <humanoid_wheel_interface_ros/MobileManipulatorDummyVisualization.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_wheel_wbc/WeightedWbc.h"

#include "humanoid_wheel_interface/motion_planner/VelocityLimiter.h"

namespace humanoid_wheel_controller
{
  using namespace ocs2;
  using namespace humanoid;
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
  class humanoidWheelController
  {
  public:
    humanoidWheelController() = default;
    ~humanoidWheelController();
    bool init(ros::NodeHandle &controller_nh, bool is_nodelet_node = false);
    bool preUpdate(const ros::Time &time);
    bool preUpdateComplete() {return isPreUpdateComplete;}
    void update(const ros::Time &time, const ros::Duration &period);
    void starting(const ros::Time &time);
  protected:

    const std::string robotName_ = "mobile_manipulator";
    
    void setupHumanoidWheelInterface(const std::string &taskFile, const std::string &urdfFile, 
                                     const std::string &libFolder);
    void computeObservationFromSensorData(const SensorData& sensorData, const vector6_t& odomData);
    void sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void setupMrt();
    void initMPC();
    void getEEPose(const vector_t& init_q, std::vector<Eigen::Vector3d>& ee_pos, std::vector<Eigen::Matrix3d>& ee_rot);
    void getTorsoPose(const vector_t& init_q, Eigen::Vector3d& torso_pos, Eigen::Matrix3d& torso_rot);
    Eigen::Vector3d cmdVelWorldToBody(const Eigen::Vector3d& cmd_vel_world, double yaw);
    Eigen::Vector3d cmdVelBodyToWorld(const Eigen::Vector3d& cmd_vel_body, double yaw);

    bool isPreUpdateComplete{false};
    ros::NodeHandle controllerNh_;
    bool is_real_{false};
    double dt_ = 0.001;
    KuavoDataBuffer<SensorData> *sensors_data_buffer_ptr_;

    ros::Publisher cmdVelPub_;
    ros::Publisher jointCmdPub_;
    ros::Subscriber odomSub_;
    ros::Subscriber sensorsDataSub_;

    vector6_t odomData_;
    std::mutex odom_mtx_;

    // log
    humanoid::TopicLogger *ros_logger_{nullptr};

    // robot param
    size_t armNum_ = 0;
    size_t lowJointNum_ = 0;

    // MPC 
    std::shared_ptr<mobile_manipulator::HumanoidWheelInterface> HumanoidWheelInterface_;
    std::shared_ptr<mobile_manipulator::MobileManipulatorDummyVisualization> robotVisualizer_;
    PinocchioInterface* pinocchioInterface_ptr_;
    mobile_manipulator::ManipulatorModelInfo manipulatorModelInfo_;
    std::shared_ptr<MRT_ROS_Interface> mrtRosInterface_;
    bool reset_mpc_{false};
    bool enable_mpc_{true};
    size_t plannedMode_ = 0;
    vector_t optimizedState_mrt_, optimizedInput_mrt_;

    // state estimate
    SystemObservation observation_wheel_;
    ros::Time current_time_, last_time_;

    // Whole Body Control
    std::shared_ptr<mobile_manipulator::WbcBase> wheel_wbc_;

    // 梯形插补加减速设置
    std::shared_ptr<mobile_manipulator::VelocityLimiter> velLimiter_;
    vector3_t cmd_vel_;
  };

} // namespace humanoid_wheel_controller
