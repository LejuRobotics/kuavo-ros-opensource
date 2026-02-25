#include <ros/ros.h>
#include <kuavo_msgs/footPose.h>
#include <kuavo_msgs/footPoseTargetTrajectories.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "grab_box/utils/singleStepControl.hpp"
#include "grab_box/common/ocs2_ros_interface.hpp"

using namespace GrabBox;

int main(int argc, char** argv) {
    ros::init(argc, argv, "foot_pose_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<kuavo_msgs::footPoseTargetTrajectories>(
        "/humanoid_mpc_foot_pose_target_trajectories", 10);

    ros::Duration(2).sleep(); // 等待一定时间以确保订阅者已经准备好

    bool collision_check = true;
    double dt = 0.4; // 迈一步的时间间隔
    vector<Vector4d> body_poses = {
        Vector4d(0.1, 0.0, 0, 0),
        Vector4d(0.1, -0.1, 0, 0),
        Vector4d(0.2, -0.1, 0, -30),
        Vector4d(0.3, 0.0, 0, 0),
        Vector4d(0.4, 0.0, 0, -30),
        Vector4d(0.5, 0.0, 0, 0),
    };

    double foot_bias = 0.1;
    kuavo_msgs::footPoseTargetTrajectories msg = get_multiple_steps_msg(body_poses, dt, foot_bias, collision_check);
    // pub.publish(msg);
    bool execute_success = false;
    while(!execute_success && ros::ok())
    {
    std::cout << "Send single foot control command until execute success.\n";
    callSingleFootCtrlSrv(msg, execute_success);
    ros::Duration(0.1).sleep();
    }
    return 0;
}
