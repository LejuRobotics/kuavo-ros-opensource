
#pragma once

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "ruckig/ruckig.hpp"

namespace ocs2 {
namespace mobile_manipulator {

// 支持自定义自由度的ruckig规划器封装类

class cmdPosePlannerWithRuckig {
public:
    cmdPosePlannerWithRuckig(int dofNum = 3);

    ~cmdPosePlannerWithRuckig() = default;

    // 设置当前和目标位置, 速度, 加速度
    void setCurrentPose(const Eigen::VectorXd& pose);
    void setTargetPose(const Eigen::VectorXd& pose);

    void setCurrentVelocity(const Eigen::VectorXd& velocity);

    void setCurrentAcceleration(const Eigen::VectorXd& acceleration);

    // 设置运动学约束
    void setVelocityLimits(const Eigen::VectorXd& max_velocity,
                          const Eigen::VectorXd& min_velocity);
    void setAccelerationLimits(const Eigen::VectorXd& max_acceleration, 
                               const Eigen::VectorXd& max_deceleration);
    void setJerkLimits(const Eigen::VectorXd& max_jerk);

    // 计算轨迹, 返回预计运动时间
    double calcTrajectory(void);

    // 获取规划结果，返回是否成功
    void getTrajectoryAtTime(double time,
                             Eigen::VectorXd& position,
                             Eigen::VectorXd& velocity,
                             Eigen::VectorXd& acceleration);

private:
    // 常规成员 
    size_t dofNum_{3};                     // 自由度数量

    Eigen::VectorXd current_pose_;          // 当前位姿
    Eigen::VectorXd current_velocity_;      // 当前速度
    Eigen::VectorXd current_acceleration_;  // 当前加速度

    Eigen::VectorXd target_pose_;           // 目标位姿

    double maxDuration_{0.0};               // 轨迹最大持续时间
    
    // ruckig 相关成员
    std::vector<ruckig::InputParameter<1>> inputVec_;
    std::vector<ruckig::Ruckig<1>> ruckigPlannerVec_;
    std::vector<ruckig::Trajectory<1>> trajectoryVec_;

};

}  // namespace mobile_manipulator
}  // namespace ocs2
