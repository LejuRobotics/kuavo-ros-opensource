#ifndef HUMANOID_PLAN_ARM_TRAJECTORY_H
#define HUMANOID_PLAN_ARM_TRAJECTORY_H
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kuavo_msgs/planArmState.h>
#include <std_srvs/Trigger.h>

namespace ocs2 {
  namespace humanoid {

    #define HAND_JOINT_TOTAL_NUM 12

    enum JointLimitation {
      JOINT_LIMIT_LOWER = 0,
      JOINT_LIMIT_UPPER = 1,
      JOINT_LIMIT_STATUS = 2
    };

    enum JointLimitStatus {
      JOINT_LIMIT_NOT_EXISTS = 0,
      JOINT_LIMIT_EXISTS = 1
    };

    class HumanoidPlanArmTrajectory {
      public:
        HumanoidPlanArmTrajectory(const std::string& name = "HumanoidPlanArmTrajectory", int joint_num = 0, const std::string& interpolate_type = "");
        virtual ~HumanoidPlanArmTrajectory() = default;
        void timerCallback(const ros::TimerEvent& event);
        virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) = 0;
        bool stopPlanArmTrajectoryCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        inline void stop() { is_stopped_ = true; stop_step_ = step_; }
        void updateTrajectoryPoint(const std::vector<double>& positions, 
                                   const std::vector<double>& velocities, 
                                   const std::vector<double>& accelerations);
        void updateTrajectoryState(const int progress, const bool is_finished);
        void updateJointState(const std::vector<double>& positions, const std::vector<double>& velocities);
        static std::string current_interpolate_type_;

      protected:
        virtual void interpolate() = 0; // interpolate the trajectory
        virtual void update() = 0; // update the joint state from the trajectory
        virtual void reset() = 0; // reset the trajectory
        virtual void initializeSpecific() = 0;
        virtual void initializeLimitations() = 0;
        void initializeCommon();
        
        std::vector<std::string> joint_names_;
        std::string name_;
        std::string interpolate_type_;
        std::string joint_state_topic_;
        std::string joint_state_unit_;
        int joint_num_ = 0;
        double step_ = 0.0;
        double stop_step_ = 0.0;
        constexpr static double dt_ = 1.0e-3; 
        constexpr static int rate_ = 1000; // default 1000 Hz
        bool is_finished_ = false;
        bool is_stopped_ = false;

        ros::NodeHandle* nh_;
        ros::NodeHandle* private_nh_;
        ros::Publisher arm_traj_pub_;
        ros::Publisher arm_traj_state_pub_;
        ros::Publisher joint_state_pub_;
        ros::ServiceServer plan_arm_traj_srv_; // need to initialize in the inherited class
        ros::ServiceServer stop_arm_traj_srv_; 
        ros::Timer timer_;
        
        bool interpolate_finished_ = false;
        sensor_msgs::JointState joint_state_;
        trajectory_msgs::JointTrajectory traj_;
        kuavo_msgs::planArmState arm_traj_state_;

        int robot_version_ = 45;
        std::vector<std::string> arm_joint_names_;
        const std::array<std::string, 2> head_joint_names_ = { "zhead_1_joint", "zhead_2_joint" };
        // 贝塞尔曲线插值器中使用弧度值，此处为手指关节限位(0 ~ 100)的转换
        constexpr static double hand_joint_lower_limit_ = 0.0;
        constexpr static double hand_joint_upper_limit_ = 1.7453;
    };
  }
}

#endif // HUMANOID_PLAN_ARM_TRAJECTORY_H
