
#pragma once

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>

namespace ocs2 {
namespace mobile_manipulator {

class VelocityLimiter {
public:
    VelocityLimiter(int vel_num);

    // 加速度限制函数
    Eigen::VectorXd limitAcceleration(const Eigen::VectorXd& desired_vel);
    void setAccelerationLimits(const Eigen::VectorXd& max_acceleration, 
                               const Eigen::VectorXd& max_deceleration);
    void setAccelerationDt(const double dt);

private:
    Eigen::VectorXd prev_cmd_vel_;
    Eigen::VectorXd max_acceleration_;
    Eigen::VectorXd max_deceleration_;
    int vel_num_;
    double acc_dt_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
