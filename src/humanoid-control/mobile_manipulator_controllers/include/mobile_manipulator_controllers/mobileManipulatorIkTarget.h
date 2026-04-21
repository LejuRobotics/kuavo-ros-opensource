#ifndef MOBILE_MANIPULATOR_IK_TARGET_H
#define MOBILE_MANIPULATOR_IK_TARGET_H

#include <pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "kuavo_msgs/armHandPose.h"
#include "kuavo_msgs/twoArmHandPoseCmd.h"
#include "ros/publisher.h"
#include "ros/service_server.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <kuavo_msgs/setMmCtrlFrame.h>
#include <kuavo_msgs/armTargetPoses.h>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "std_msgs/Int32.h"
#include <Eigen/Dense>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorSpatialKinematics.h>
#include <mutex>

namespace mobile_manipulator_controller
{
using namespace ocs2;

enum class FrameType
{
  CurrentFrame = 0,    // keep current frame
  WorldFrame = 1,     
  LocalFrame = 2,
  VRFrame = 3,
  MmWorldFrame = 4
};

struct IkCmd
{
    Eigen::Vector3d pos;     // hand pos
    Eigen::Quaterniond quat; // hand quaternion
};

struct BasePoseCmd
{
  int dim;
  Eigen::Matrix<double, 6, 1> pose;
};

class MobileManipulatorIkTarget {
  public:
    MobileManipulatorIkTarget(ros::NodeHandle& nodeHandle, const std::string& robotName);
    ~MobileManipulatorIkTarget() = default;

    // 禁用拷贝构造和赋值
    MobileManipulatorIkTarget(const MobileManipulatorIkTarget&) = delete;
    MobileManipulatorIkTarget& operator=(const MobileManipulatorIkTarget&) = delete;

    // 允许移动构造和赋值
    MobileManipulatorIkTarget(MobileManipulatorIkTarget&&) = default;
    MobileManipulatorIkTarget& operator=(MobileManipulatorIkTarget&&) = default;

    void run();
    void stop();

    // 公共接口方法
    bool setQuest3Utils(bool useQuest3Utils);
    bool setFrameType(FrameType frameType);
    FrameType getFrameType() const;
    bool isObservationReceived() const;
    bool isHumanoidObservationReceived() const;
    int getEffTrajReceived() const;
    bool getTargetTrajectories(TargetTrajectories& targetTrajectories);
    void setEnableHumanoidObservationCallback(bool enable){enableHumanoidObservationCallback_ = enable;}
    bool getEnableHumanoidObservationCallback() const {return enableHumanoidObservationCallback_;}
    bool setHumanoidObservationByMmState(const vector_t& mmState);

  private:
    // 回调函数
    bool setQuest3UtilsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool setFrameTypeCallback(kuavo_msgs::setMmCtrlFrame::Request& req, kuavo_msgs::setMmCtrlFrame::Response& res);
    bool getFrameTypeCallback(kuavo_msgs::setMmCtrlFrame::Request& req, kuavo_msgs::setMmCtrlFrame::Response& res);
    void publishStatus(const ros::TimerEvent& event);
    void basePoseCmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void ikCmdCallback(const kuavo_msgs::twoArmHandPoseCmd::ConstPtr& msg);
    void mmEndEffectorTrajectoryCallback(const kuavo_msgs::armTargetPoses::ConstPtr& msg);

    // 观察回调函数
    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);
    void humanoidObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

    // 核心功能方法
    TargetTrajectories goalPoseToTargetTrajectories(const IkCmd& cmd_l, const IkCmd& cmd_r, const SystemObservation& observation);
    ocs2::TargetTrajectories generateTwoHandTargetTrajectories(const ocs2::vector_array_t& poses, const ocs2::scalar_array_t& times);
    vector_t getMMEefPose(const vector_t& state);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transTargetToLocalFrame(const FrameType& frameType, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat, const SystemObservation& observation);

    // 坐标变换方法
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromVRFrameToMmWorld(
        const Eigen::Vector3d& p_be, const Eigen::Quaterniond& quat_be, const SystemObservation& observation);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromLocalFrameToWorld(
        const Eigen::Vector3d& p_le, const Eigen::Quaterniond& quat_le, const SystemObservation& observation);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromWorldFrameToMmWorld(
        const Eigen::Vector3d& p_we, const Eigen::Quaterniond& quat_we, const SystemObservation& observation);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromMmWorldFrameToLocalFrame(
        const Eigen::Vector3d& p_mw, const Eigen::Quaterniond& quat_mw, const SystemObservation& observation);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromVRFrameToLocalFrame(
        const Eigen::Vector3d& p_le, const Eigen::Quaterniond& quat_le, const SystemObservation& observation);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromWorldFrameToLocalFrame(
        const Eigen::Vector3d& p_we, const Eigen::Quaterniond& quat_we, const SystemObservation& observation);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> transPoseFromLocalFrameToMmWorld(
        const Eigen::Vector3d& p_le, const Eigen::Quaterniond& quat_le, const SystemObservation& observation);

    // 初始化方法
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeTimers();
    void loadParameters();
    void setupMobileManipulatorInterface();

  private:
    // ROS相关
    ros::NodeHandle& nodeHandle_;
    ros::Timer status_publish_timer_;
    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
    
    // 订阅者
    ::ros::Subscriber observationSubscriber_;
    ::ros::Subscriber humanoidObservationSubscriber_;
    ::ros::Subscriber ikCmdSubscriber_;
    ::ros::Subscriber mmEndEffectorTrajectorySubscriber_;
    ::ros::Subscriber basePoseCmdSubscriber_;
    
    // 发布者
    ::ros::Publisher effTrajReceivedPublisher_;
    
    // 服务
    ::ros::ServiceServer setQuest3UtilsService_;
    ::ros::ServiceServer setFrameTypeService_;
    ::ros::ServiceServer getFrameTypeService_;

    // 状态变量
    SystemObservation latestObservation_;
    SystemObservation latestHumanoidObservation_;
    bool observationReceived_ = false;
    bool humanoidObservationReceived_ = false;
    bool enableHumanoidObservationCallback_ = true;
    bool newBasePoseReceived_ = false;
    BasePoseCmd basePoseCmd_;
    bool use_quest3_utils_ = false;
    FrameType frameType_ = FrameType::MmWorldFrame;
    double comHeight_ = 0.0;
    int dof_target_pose_ = 14;
    int effTrajReceived_ = 0;

    // 机器人接口
    std::shared_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> mobileManipulatorInterface_;
    std::unique_ptr<PinocchioInterface> pinocchioInterface_ptr_;
    std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping> pinocchioMappingPtr_;
    std::shared_ptr<PinocchioEndEffectorSpatialKinematics> eeSpatialKinematicsPtr_;
    ocs2::mobile_manipulator::ManipulatorModelInfo info_;

    // 局部坐标系下的中间变量存储
    struct LocalFrameData {
        IkCmd leftHandPose;      // 左手局部坐标系下的姿态
        IkCmd rightHandPose;     // 右手局部坐标系下的姿态
        bool isValid;            // 数据是否有效
        ros::Time timestamp;     // 数据时间戳
        
        LocalFrameData() : isValid(false) {}
    };

    // 局部坐标系下的数据存储
    LocalFrameData localFrameData_;
    mutable std::mutex localDataMutex_;

    // 配置参数
    std::string robotName_;
    bool running_ = false;
    TargetTrajectories targetTrajectories_;
    bool targetTrajectoriesReceived_ = false;
    mutable std::mutex targetTrajectoriesMutex_;
};

}  // namespace mobile_manipulator_controller

#endif // MOBILE_MANIPULATOR_IK_TARGET_H 