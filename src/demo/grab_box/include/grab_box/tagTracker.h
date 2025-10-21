#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "grab_box/utils/poseTransformer.h"
#include "kuavo_msgs/robotHeadMotionData.h"
#include <thread>
#include <mutex>
#include <kuavo_msgs/setTagId.h>
#include "kuavo_msgs/tagDataArray.h"
#include <yaml-cpp/yaml.h>
#include <vector>

namespace autoHeadChase {

using namespace std;

class TagTracker {
public:
    TagTracker();
    void run();  // Runs the ROS node

private:
    // Callback functions for subscribers
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void tagInfoCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    // Service handlers
    bool oneTimeTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool continuousTrackService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool setTargetTagIdService(kuavo_msgs::setTagId::Request& req, kuavo_msgs::setTagId::Response& res);
    bool delTargetTagIdService(kuavo_msgs::setTagId::Request& req, kuavo_msgs::setTagId::Response& res);

    // Internal functions
    void updateTagWorldPose();
    void calculateHeadOrientation();
    void publishHeadOrientationCommand(double pitch, double yaw);

    void publishTagData();

    bool checkSafty(const Eigen::VectorXd&  tag_pose_world, const vector<double>& safe_space);

    // Node handles, subscribers, and publishers
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber tag_info_sub_;
    ros::Publisher head_orientation_pub_;
    ros::ServiceServer one_time_track_srv_;
    ros::ServiceServer continuous_track_srv_;
    ros::Publisher tag_world_pose_pub_;
    ros::ServiceServer set_target_tag_id_srv_; 
    ros::ServiceServer del_target_tag_id_srv_;
    ros::Publisher tag_data_pub_;

    // State variables
    Eigen::VectorXd robot_pose_world_;  // Robot pose in the world frame (7-dim: pos + quat)
    Eigen::VectorXd tag_pose_robot_;    // Tag pose in the robot frame (7-dim: pos + quat)
    Eigen::VectorXd tag_pose_world_;    // Tag pose in the world frame (7-dim: pos + quat)
    std::map<int, Eigen::VectorXd> tag_data_;

    bool is_continuous_tracking_;  // Flag to manage continuous tracking
    ros::Timer tracking_timer_;    // Timer for continuous tracking
    double pitch0  = 0.6105555;
    int robot_version_ = 40;
    double heigjht_neck_motor_offset_ = 0.5;
    double head_link_ = 0.05449;
    bool odom_received_ = false;
    bool tag_info_received_ = false;
    int target_tag_id_;
    std::mutex data_mutex_;
    vector<double> safe_space_;

};

}  // namespace autoHeadChase
