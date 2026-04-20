
#pragma once

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "ruckig/ruckig.hpp"
#include "humanoid_wheel_interface/motion_planner/cmdPosePlannerWithRuckig.h"

#include "humanoid_wheel_interface/estimators/CentralDifferenceDifferentiator.h"

namespace ocs2 {
namespace mobile_manipulator {

// 用于调度ruckig规划器的时间同步类
class posePlannerTimedScheduler {
public:
    posePlannerTimedScheduler() = default;
    ~posePlannerTimedScheduler() = default;

    // 获取时间同步的ruckig位置规划器列表
    const std::vector<std::shared_ptr<cmdPosePlannerWithRuckig>>& getTimedPlannerPosePtrVec() const;

    // 添加时间同步的ruckig位置规划器
    void addTimedPlannerPosePtr(const std::shared_ptr<cmdPosePlannerWithRuckig>& plannerPtr);

    // 设置时间同步器中的状态信息
    void setTimedPlannerStates(const std::vector<Eigen::VectorXd>& currentPose);
    void setTimedPlanner_dStates(const std::vector<Eigen::VectorXd>& currentVelocity);
    void setTimedPlanner_ddStates(const std::vector<Eigen::VectorXd>& currentAcceleration);

    // 获取时间同步器中指定规划器的状态信息
    void getTimedPlannerStates(int8_t plannerIndex, 
                               Eigen::VectorXd& currentPose, 
                               Eigen::VectorXd& currentVelocity, 
                               Eigen::VectorXd& currentAcceleration);

    // 计算时间同步的轨迹，返回预计运动时间
    double calcTimedTrajectory(int8_t plannerIndex, Eigen::VectorXd cmdVec, double desiredTime = 0.0);

    // 更新指定索引的速度限制
    void updateTimedPlannerVelocityLimits(int8_t plannerIndex,
                                          const Eigen::VectorXd& max_velocity,
                                          const Eigen::VectorXd& min_velocity);

    // 更新指定索引的加速度限制
    void updateTimedPlannerAccelerationLimits(int8_t plannerIndex,
                                              const Eigen::VectorXd& max_acceleration,
                                              const Eigen::VectorXd& max_deceleration);

    // 更新指定索引的加加速度限制
    void updateTimedPlannerJerkLimits(int8_t plannerIndex,
                                      const Eigen::VectorXd& max_jerk);

    // 设置指定规划器的同步模式
    void setTimedPlannerSyncMode(int8_t plannerIndex, bool isSync);

    // 获取指定规划器的自由度数量
    int getTimedPlannerDofNum(int8_t plannerIndex) const;

    // 获取指定规划器在指定时间的轨迹信息
    void getTimedPlannerTrajectoryAtTime(int8_t plannerIndex,
                                        double time,
                                        Eigen::VectorXd& position,
                                        Eigen::VectorXd& velocity,
                                        Eigen::VectorXd& acceleration);
    
    int getPlannersNum() const { return static_cast<int>(timedPlannerPosePtrVec_.size()); }

    void setDiffDt(double dt) { diffDt_ = dt; }

private:
    std::vector<std::shared_ptr<cmdPosePlannerWithRuckig>> timedPlannerPosePtrVec_;  // 时间同步的ruckig位置规划器列表
    std::vector<std::atomic<bool>> isCalcTraj_; // 记录每个规划器是否刚完成轨迹计算, 且未被读取

    // 默认采用差分获取速度和加速度
    std::vector<CentralDifferenceDifferentiator> stateDiffVec_;
    double diffDt_{0.01};   // 默认100Hz

    // 轨迹信息记录
    std::vector<Eigen::VectorXd> currentPos_;
    std::vector<Eigen::VectorXd> currentVel_;
    std::vector<Eigen::VectorXd> currentAcc_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2