#include "humanoid_plan_arm_trajectory.h"

namespace ocs2 {
  namespace humanoid {
    std::string HumanoidPlanArmTrajectory::current_interpolate_type_ = "cubic_spline";

    HumanoidPlanArmTrajectory::HumanoidPlanArmTrajectory(const std::string& name, int joint_num, const std::string& interpolate_type) : joint_num_(joint_num), name_(name), interpolate_type_(interpolate_type) {
      joint_state_ = sensor_msgs::JointState();
      traj_ = trajectory_msgs::JointTrajectory();
      arm_traj_state_ = kuavo_msgs::planArmState();
    }

    void HumanoidPlanArmTrajectory::initializeCommon() {
      nh_->getParam("joint_state_topic", joint_state_topic_);
      nh_->getParam("joint_state_unit", joint_state_unit_);
      nh_->getParam("robot_version", robot_version_);
      if (robot_version_ >= 40) {
        arm_joint_names_ = {
          "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
          "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"
        };
      } else if (robot_version_ >= 10 && robot_version_ < 30) {
        arm_joint_names_ = {
          "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
          "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
        };
      }
      arm_traj_pub_ = nh_->advertise<trajectory_msgs::JointTrajectory>(interpolate_type_ + "/arm_traj", 10);
      arm_traj_state_pub_ = nh_->advertise<kuavo_msgs::planArmState>(interpolate_type_ + "/arm_traj_state", 10);
      joint_state_pub_ = nh_->advertise<sensor_msgs::JointState>(joint_state_topic_, 10);
      timer_ = nh_->createTimer(ros::Duration(1.0 / rate_), &HumanoidPlanArmTrajectory::timerCallback, this);
      stop_arm_traj_srv_ = nh_->advertiseService(interpolate_type_ + "/stop_plan_arm_trajectory", &HumanoidPlanArmTrajectory::stopPlanArmTrajectoryCallback, this);
    }

    void HumanoidPlanArmTrajectory::timerCallback(const ros::TimerEvent& event) {
      if (interpolate_type_ != current_interpolate_type_ || joint_num_ == 0 || is_finished_ == true) {
        return;
      }
      update();
      // joint_state_pub_.publish(joint_state_);
      arm_traj_pub_.publish(traj_);
      arm_traj_state_pub_.publish(arm_traj_state_);
    }

    bool HumanoidPlanArmTrajectory::stopPlanArmTrajectoryCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
      stop();
      res.success = true;
      return true;
    }

    void HumanoidPlanArmTrajectory::updateTrajectoryPoint(const std::vector<double>& positions, 
                                                    const std::vector<double>& velocities, 
                                                    const std::vector<double>& accelerations) {
        if (traj_.points.empty()) {
            traj_.points.resize(1);
        }

        traj_.header.stamp = ros::Time::now();
        traj_.joint_names = joint_names_;
        auto& point = traj_.points[0];
        point.positions = positions;
        point.velocities = velocities;
        point.accelerations = accelerations;
        point.time_from_start = ros::Duration(step_);
    }

    void HumanoidPlanArmTrajectory::updateJointState(const std::vector<double>& positions, const std::vector<double>& velocities) {
      joint_state_.header.stamp = ros::Time::now();
      joint_state_.name = joint_names_;
      joint_state_.position.resize(joint_num_);
      joint_state_.velocity.resize(joint_num_);
      if (joint_state_unit_ == "deg") {
        for (int i = 0; i < joint_num_; i++) {
          joint_state_.position[i] = positions[i] * 180.0 / M_PI;
          joint_state_.velocity[i] = velocities[i] * 180.0 / M_PI;
        }
      } else { // default is rad
        joint_state_.velocity = velocities;
        joint_state_.position = positions;
      }
    }

    void HumanoidPlanArmTrajectory::updateTrajectoryState(const int progress, const bool is_finished) {
      arm_traj_state_.progress = progress;
      arm_traj_state_.is_finished = is_finished;
    }
  }
}