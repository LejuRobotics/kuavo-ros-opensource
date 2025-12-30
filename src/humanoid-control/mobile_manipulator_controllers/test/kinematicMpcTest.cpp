#include <ros/ros.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/endEffectorData.h"
#include <std_msgs/Float64MultiArray.h>

#include <humanoid_interface/gait/ModeSequenceTemplate.h>
#include "humanoid_interface_ros/gait/ModeSequenceTemplateRos.h"

using namespace ocs2;
struct HandPose 
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
};

bool changeArmCtrlModeSrv(int mode)
{
  const std::string service_name = "/humanoid_change_arm_ctrl_mode";
  ros::NodeHandle nh;

  // 等待服务可用
  if (!ros::service::waitForService(service_name, ros::Duration(5))) {
    ROS_ERROR("Service %s not available", service_name.c_str());
    return false;
  }

  // 创建服务代理
  ros::ServiceClient client = nh.serviceClient<kuavo_msgs::changeArmCtrlMode>(service_name);
  kuavo_msgs::changeArmCtrlMode srv;
  srv.request.control_mode = mode;

  // 调用服务
  if (client.call(srv)) {
    return true; // 服务调用成功
  } else {
    ROS_ERROR("Failed to call service %s", service_name.c_str());
    return false; // 服务调用失败
  }
}

enum class GraspMode
{
  TOGOTHER,
  ARM_ONLY,
};

class KinematicMpcTest
{
public:
  KinematicMpcTest(ros::NodeHandle& nodeHandle, std::string robotName)
  : nodeHandle_(nodeHandle)
  {
    targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle_, robotName));
    std::cout << "KinematicMpcTest initialized" << std::endl;
    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
      if(!observationReceived_)
      {
        observationReceived_ = true;
        baseDim_ = latestObservation_.state.size() - 14;
        std::cout << "baseDim: " << baseDim_ << std::endl;
        q_matrix_diagonal_high_ = std::vector<double>(baseDim_, 20.0);
        q_matrix_diagonal_low_ = std::vector<double>(baseDim_, 10.01);
        std::cout << "q_matrix_diagonal_high_: " << Eigen::Map<const Eigen::VectorXd>(q_matrix_diagonal_high_.data(), q_matrix_diagonal_high_.size()).transpose() << std::endl;
        // std::cout << "q_matrix_diagonal_low_: " << q_matrix_diagonal_low_ << std::endl;
      }
    };
    observationSubscriber_ = nodeHandle_.subscribe<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1, observationCallback);
    auto eePoseCallback = [this](const kuavo_msgs::endEffectorData::ConstPtr& msg) {
      real_hand_pose_l.pos << msg->position[0], msg->position[1], msg->position[2];
      real_hand_pose_l.quat = Eigen::Quaterniond(msg->position[6], msg->position[3], msg->position[4], msg->position[5]);
      real_hand_pose_r.pos << msg->position[7], msg->position[8], msg->position[9];
      real_hand_pose_r.quat = Eigen::Quaterniond(msg->position[13], msg->position[10], msg->position[11], msg->position[12]);
    };
    eePoseSubscriber_ = nodeHandle_.subscribe<kuavo_msgs::endEffectorData>("/humanoid_ee_State", 1, eePoseCallback);
    dynamicQMatrixPublisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(robotName + "_q_matrix_diagonal", 1);
    modeSequenceTemplatePublisher_ = nodeHandle_.advertise<ocs2_msgs::mode_schedule>("/humanoid_mpc_mode_schedule", 10, true);

    while(!nodeHandle_.hasParam("/com_height"))
    {
      ROS_ERROR_STREAM("com_height parameter is NOT found, waiting for 0.1s.");
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("com_height parameter is founded.");
    nodeHandle_.getParam("/com_height", comHeight_);
    std::cout << "[KinematicMpcTest] comHeight: " << comHeight_<<std::endl;

    // gait
    std::string gaitCommandFile;
    nodeHandle_.getParam("/gaitCommandFile", gaitCommandFile);
    ROS_INFO_STREAM("/KinematicMpcTest node is setting up ...");
    std::vector<std::string> gaitList;
    ocs2::loadData::loadStdVector(gaitCommandFile, "list", gaitList, false);
    gait_map_.clear();
    for (const auto &gaitName : gaitList)
    {
      gait_map_.insert({gaitName, ocs2::humanoid::loadModeSequenceTemplate(gaitCommandFile, gaitName, false)});
    }
    std::cout << "Initialized observation subscriber" << std::endl;
  }

  void run()
  {
    while(!observationReceived_)
    {
      std::cout << "Waiting for observation" << std::endl;
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    Eigen::VectorXd torso_pose = Eigen::VectorXd::Zero(baseDim_);// x, y, yaw
    HandPose pose_l, pose_r;
    auto q = Eigen::Quaternion<scalar_t>(0.707, 0, -0.707, 0);
    q.normalize();
    pose_l.pos << 0.4, 0.3, 0.4;
    pose_l.quat = q;
    pose_r.pos << 0.4, -0.3, 0.4;
    pose_r.quat = q;
    ros::Rate loop_rate(10);
    changeArmCtrlModeSrv(2);
    if(isTargetReachable(pose_l.pos, pose_r.pos, latestObservation_, 0.6))
    {
      torso_pose << latestObservation_.state.segment(0, baseDim_);
      publishGaitTemplate("stance");
      pubQMatrix(q_matrix_diagonal_high_);
    }
    else{
      std::cout << "Target not reachable" << std::endl;
      publishGaitTemplate("walk");
      pubQMatrix(q_matrix_diagonal_low_);
      ros::Duration(1.0).sleep();
      publishGaitTemplate("walk");
    }
    bool stoped = false;
    while(ros::ok() && !stoped)
    {
      // std::cout << "running" << std::endl;
      ros::spinOnce();
      loop_rate.sleep();
      if(isEefTargetReached(pose_l.pos, pose_r.pos, 0.05))
      {
        publishGaitTemplate("stance");
        pubQMatrix(q_matrix_diagonal_high_);
        stoped = true;
        std::cout << "Target reached!" << std::endl;
      }
      torso_pose << latestObservation_.state.segment(0, baseDim_);
      auto target = goalPoseToTargetTrajectories(torso_pose, pose_l, pose_r, latestObservation_);
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(target); 
    }
  }

private:
  TargetTrajectories goalPoseToTargetTrajectories(const Eigen::VectorXd& torso_pose, const HandPose& pose_l, const HandPose& pose_r, const SystemObservation& observation) 
  {
    int baseDim = torso_pose.size();
    // time trajectory
    const scalar_array_t timeTrajectory{observation.time};
    // state trajectory: 3 + 4 for desired position vector and orientation quaternion
    auto l_pose = (vector_t(7) << pose_l.pos, pose_l.quat.coeffs()).finished(); 
    auto r_pose = (vector_t(7) << pose_r.pos, pose_r.quat.coeffs()).finished();
    vector_t target;
    if(baseDim > 0)
      target = (vector_t(baseDim+14) << torso_pose, l_pose, r_pose).finished();
    else
      target = (vector_t(14) << l_pose, r_pose).finished();
    const vector_array_t stateTrajectory{target};
    // input trajectory
    const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  void pubQMatrix(std::vector<double> q_matrix_diagonal)
  {
    std_msgs::Float64MultiArray msg;
    msg.data = q_matrix_diagonal;
    dynamicQMatrixPublisher_.publish(msg);
  }

  double getTargetDistance(const Eigen::Vector3d& target, const SystemObservation& observation)
  {
    //TODO: only fit for wheelbase robot
    Eigen::Vector3d base_pos = Eigen::Vector3d::Zero();
    base_pos.head<2>() = observation.state.segment<2>(0);
    double norm = (target - base_pos).norm();
    return norm;
  }

  bool isTargetReachable(const Eigen::Vector3d& target_l, const Eigen::Vector3d& target_r, const SystemObservation& observation, double threshold=0.5)
  {
    double distance_l = getTargetDistance(target_l, observation);
    double distance_r = getTargetDistance(target_r, observation);
    std::cout << "distance_l: " << distance_l << ", distance_r: " << distance_r << std::endl;
    return (distance_l < threshold) && (distance_r < threshold);
  }

  bool isEefTargetReached(const Eigen::Vector3d& target_l, const Eigen::Vector3d& target_r, double threshold=0.01)
  {
    // if(baseDim_ == 3)
    {
      real_hand_pose_l.pos.z() -= comHeight_;
      real_hand_pose_r.pos.z() -= comHeight_;
    }
    double distance_l = (real_hand_pose_l.pos - target_l).norm();
    double distance_r = (real_hand_pose_r.pos - target_r).norm();
    std::cout << "dis l: " << (real_hand_pose_l.pos - target_l).transpose() << std::endl;
    std::cout << "dis r: " << (real_hand_pose_r.pos - target_r).transpose() << std::endl;
    std::cout << "eef distance_l: " << distance_l << ", eef distance_r: " << distance_r << std::endl;
    return (distance_l < threshold) && (distance_r < threshold);
  }

  void publishGaitTemplate(const std::string &gaitName)
  {
    // 发布对应的gait模板
    ocs2::humanoid::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gaitName);
    modeSequenceTemplatePublisher_.publish(ocs2::humanoid::createModeSequenceTemplateMsg(modeSequenceTemplate));
  }

private:
  std::map<std::string, ocs2::humanoid::ModeSequenceTemplate> gait_map_;
  ros::NodeHandle nodeHandle_;
  ros::Subscriber observationSubscriber_;
  ros::Subscriber eePoseSubscriber_;
  ros::Publisher dynamicQMatrixPublisher_;
  ros::Publisher modeSequenceTemplatePublisher_;
  SystemObservation latestObservation_;
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
  bool observationReceived_ = false;
  std::vector<double> q_matrix_diagonal_high_;
  std::vector<double> q_matrix_diagonal_low_;
  HandPose real_hand_pose_l, real_hand_pose_r;
  double comHeight_;
  int baseDim_;
};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "kinematic_mpc_test");
  ros::NodeHandle nodeHandle;
  KinematicMpcTest kinematicMpcTest(nodeHandle, "mobile_manipulator");
  kinematicMpcTest.run();
  return 0;
}