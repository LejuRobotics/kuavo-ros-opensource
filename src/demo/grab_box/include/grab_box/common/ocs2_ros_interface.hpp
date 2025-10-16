#include <ros/ros.h>
#include <Eigen/Core>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <std_msgs/Float64MultiArray.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/twoArmHandPoseCmdSrv.h"
#include "kuavo_msgs/ikSolveParam.h"
#include "kuavo_msgs/headBodyPose.h"
#include "kuavo_msgs/changeArmCtrlMode.h"
#include "kuavo_msgs/footPoseTargetTrajectoriesSrv.h"
#include "kuavo_msgs/footPoseTargetTrajectories.h"
#include "kuavo_msgs/changeTorsoCtrlMode.h"

#pragma once

namespace GrabBox
{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> HandPose;
  typedef std::pair<HandPose, HandPose> TwoHandPose;
  typedef std::vector<TwoHandPose> TwoHandPoseTrajectory;
  typedef std::vector<std::pair<TwoHandPose, Eigen::Vector3d>> TwoHandPoseWithForceTrajectory;//pose+force

  /**
   * @brief 归一化角度，将角度限制在 [-180, 180] 之间
   * @param yaw 输入角度 [deg]
   */
  double normalizedYaw(double yaw)
  {
    while (yaw > 180.0)
      yaw -= 2*180.0;
    while(yaw < -180.0)
      yaw += 2*180.0;
    return yaw;
  }

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

  int getArmControlModeSrv ()
  {
    const std::string service_name = "/humanoid_get_arm_ctrl_mode";
    ros::NodeHandle nh;
// 等待服务可用
    if (!ros::service::waitForService(service_name, ros::Duration(5))) {
      ROS_ERROR("Service %s not available", service_name.c_str());
      return false;
    }

    // 创建服务代理
    ros::ServiceClient client = nh.serviceClient<kuavo_msgs::changeArmCtrlMode>(service_name);
    kuavo_msgs::changeArmCtrlMode srv;
    srv.request.control_mode = 0;

    // 调用服务
    if (client.call(srv)) {
      return srv.response.mode; // 服务调用成功
    } else {
      ROS_ERROR("Failed to call service %s", service_name.c_str());
      return -1; // 服务调用失败
    }
  }

  bool limitArmControlMode()
  {
    int mode = getArmControlModeSrv();
    if (mode == 2){
      changeArmCtrlModeSrv(0);
      ROS_INFO("Arm control mode changed to 0");
      return true;
    }
    else if (mode == 1){
      changeArmCtrlModeSrv(0);

      ROS_INFO("Arm control mode is already 1");
    }
    else{
      changeArmCtrlModeSrv(0);

      ROS_INFO("Arm control mode is already 0");
    }

    return true;
  }

  bool callSingleFootCtrlSrv(const kuavo_msgs::footPoseTargetTrajectories &msg, bool &executed)
  {
    const std::string service_name = "/humanoid_mpc_foot_pose_target_trajectories_srv";
    ros::NodeHandle nh;

    // 等待服务可用
    if (!ros::service::waitForService(service_name, ros::Duration(5))) {
      ROS_ERROR("Service %s not available", service_name.c_str());
      return false;
    }

    // 创建服务代理
    ros::ServiceClient client = nh.serviceClient<kuavo_msgs::footPoseTargetTrajectoriesSrv>(service_name);
    kuavo_msgs::footPoseTargetTrajectoriesSrv srv;
    srv.request.foot_pose_target_trajectories = msg;

    executed = false;
    // 调用服务
    if (client.call(srv)) {
      executed = srv.response.success;
      return true; // 服务调用成功
    } else {
      ROS_ERROR("Failed to call service %s", service_name.c_str());
      return false; // 服务调用失败
    }
  }

  bool changeKinematicMpcControlMode(int mode)
  {
    const std::string service_name = "/mobile_manipulator_mpc_control";
    ros::NodeHandle nh;
    if (!ros::service::waitForService(service_name, ros::Duration(1))) {
      ROS_ERROR("Service %s not available", service_name.c_str());
      return false;
    }
    ros::ServiceClient client = nh.serviceClient<kuavo_msgs::changeTorsoCtrlMode>(service_name);
    kuavo_msgs::changeTorsoCtrlMode srv;
    srv.request.control_mode = mode;

    // 调用服务
    if (client.call(srv)) {
      return true; // 服务调用成功
    } else {
      ROS_ERROR("Failed to call service %s", service_name.c_str());
      return false; // 服务调用失败
    }
  }

  bool resetMpcService()
  {
    const std::string service_name = "/reset_mm_mpc";
    ros::NodeHandle nh;
    if (!ros::service::waitForService(service_name, ros::Duration(1))) {
      ROS_ERROR("Service %s not available", service_name.c_str());
      return false;
    }
    ros::ServiceClient client = nh.serviceClient<kuavo_msgs::changeTorsoCtrlMode>(service_name);
    kuavo_msgs::changeTorsoCtrlMode srv;
    srv.request.control_mode = 0;
    if (client.call(srv)) {
      return true; // 服务调用成功
    } else {
      ROS_ERROR("Failed to call service %s", service_name.c_str());
      return false; // 服务调用失败
    }
  }

  bool resetMpcMrtService()
  {
    const std::string service_name = "/mobile_manipulator_reset_mpc_mrt";
    ros::NodeHandle nh;
    if (!ros::service::waitForService(service_name, ros::Duration(1))) {
      ROS_ERROR("Service %s not available", service_name.c_str());
      return false;
    }
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(service_name);
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (client.call(srv)) {
      return true; // 服务调用成功
    } else {
      ROS_ERROR("Failed to call service %s", service_name.c_str());
      return false; // 服务调用失败
    }
  }

  bool enableBasePitchLimit(bool enable)
  {
    const std::string service_name = "/humanoid/mpc/enable_base_pitch_limit";
    ros::NodeHandle nh;
    if (!ros::service::waitForService(service_name, ros::Duration(2))) {
      ROS_ERROR("Service %s not available", service_name.c_str());
      return false;
    }
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>(service_name);
    std_srvs::SetBool srv;
    srv.request.data = enable;
    if (client.call(srv)) {
      if (srv.response.success) {
        ROS_INFO("Successfully %s base pitch limit: %s", enable ? "enabled" : "disabled", srv.response.message.c_str());
        return true;
      } else {
        ROS_ERROR("Failed to %s base pitch limit: %s", enable ? "enable" : "disable", srv.response.message.c_str());
        return false;
      }
    } else {
      ROS_ERROR("Failed to call service %s", service_name.c_str());
      return false;
    }
  }

  std_msgs::Float64MultiArray getEefWrenchCmdMsg(const Vector6d &wrench_l, const Vector6d &wrench_r)
  {
    std_msgs::Float64MultiArray wrench_msg;

    std_msgs::MultiArrayDimension dim;
    dim.label = "";
    dim.size = 12;
    dim.stride = 12;
    wrench_msg.layout.dim.push_back(dim);

    wrench_msg.layout.data_offset = 0;
    wrench_msg.data.resize(12, 0.0);

    for(int i=0;i<6;i++)
    {
      wrench_msg.data[i] = wrench_l(i);
      wrench_msg.data[i+6] = wrench_r(i);
    }

    return std::move(wrench_msg);
  }

  /**
   * @brief 构造关节状态消息
   * @param q_arm 关节位置
   * @param dq_arm 关节速度
   */
  sensor_msgs::JointState getJointStatesMsg(const std::vector<double>& q_arm, const std::vector<double>& dq_arm)
  {
    if(q_arm.size() != dq_arm.size()) // 关节数不一致
      ROS_ERROR("q_arm, dq_arm size is not equal");
    sensor_msgs::JointState msg;
    msg.name.resize(q_arm.size());
    for (int i = 0; i < q_arm.size(); ++i) {
      msg.name[i] = "arm_joint_" + std::to_string(i + 1);
    }
    msg.header.stamp = ros::Time::now();
    
    // 假设 q_arm 的大小已符合
    msg.position.resize(q_arm.size());
    msg.velocity.resize(q_arm.size());
    for (size_t i = 0; i < q_arm.size(); ++i) {
      msg.position[i] = 180.0 / M_PI * q_arm[i]; // 转换为度      
      msg.velocity[i] = 180.0 / M_PI * dq_arm[i]; // 转换为度/s
    }
    
    return std::move(msg);
  }

  /**
   * @brief 构造关节状态消息
   * @param q_arm 关节位置
   */
  sensor_msgs::JointState getJointStatesMsg(const std::vector<double>& q_arm)
  {
    return getJointStatesMsg(q_arm, std::vector<double>(q_arm.size(), 0.0));    
  }

  kuavo_msgs::headBodyPose getHeadBodyPoseMsg(const std::vector<double>& q_torso) {
    if(q_torso.size() != 6) // xyz, pitch, yaw, roll
      ROS_ERROR("q_torso size is not 6");
    kuavo_msgs::headBodyPose msg;
    msg.head_pitch = 0;
    msg.head_yaw = 0;
    msg.body_yaw = 0;      
    // 限制 body_pitch 的值
    msg.body_pitch = std::max(3 * M_PI / 180.0, std::min(q_torso[4], 40 * M_PI / 180.0));      
    msg.body_x = 0.0;
    msg.body_y = 0.0;
    // 限制 body_height 的值
    msg.body_height = std::max(-0.4, std::min(q_torso[2], 0.2));
    return msg;
  }

  visualization_msgs::Marker constructBoxMarker(const Eigen::Vector3d& boxPoint, const Eigen::Vector3d& boxSize, const Eigen::Vector4d& quat
    , double alpha = 0.6, const std::vector<double>& color = {0, 1, 0})
  {
    if (boxPoint.size() != 3)
    {
        ROS_ERROR("Invalid pos, cannot construct marker");
        return visualization_msgs::Marker();
    }
    if (boxSize.size() != 3)
    {
        ROS_ERROR("Invalid box_size, cannot construct marker");
        return visualization_msgs::Marker();
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = boxSize[0];
    marker.scale.y = boxSize[1];
    marker.scale.z = boxSize[2];

    marker.color.a = alpha;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];

    marker.header.stamp = ros::Time::now();

    marker.pose.position.x = boxPoint[0];
    marker.pose.position.y = boxPoint[1];
    marker.pose.position.z = boxPoint[2];

    marker.pose.orientation.x = quat[0];
    marker.pose.orientation.y = quat[1];
    marker.pose.orientation.z = quat[2];
    marker.pose.orientation.w = quat[3];

    return marker;
  }

  /**
   * @brief single hand control
   *  */ 
  ocs2::TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const ocs2::SystemObservation& observation) {
    // time trajectory
    const ocs2::scalar_array_t timeTrajectory{observation.time};
    // state trajectory: 3 + 4 for desired position vector and orientation quaternion
    const ocs2::vector_t target = (ocs2::vector_t(7) << position, orientation.coeffs()).finished();
    const ocs2::vector_array_t stateTrajectory{target};
    // input trajectory
    const ocs2::vector_array_t inputTrajectory{ocs2::vector_t::Zero(observation.input.size())};

    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

} // namespace GrabBox