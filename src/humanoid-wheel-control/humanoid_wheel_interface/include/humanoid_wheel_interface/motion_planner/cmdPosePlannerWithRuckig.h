
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
    cmdPosePlannerWithRuckig(int dofNum, bool isSync = false);

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
    double calcTrajectory(double desiredTime = 0.0);

    // 非同步的预设时间规划
    double calcTrajectoryNonSync(double desiredTime);

    // 获取规划结果，返回是否成功
    void getTrajectoryAtTime(double time,
                             Eigen::VectorXd& position,
                             Eigen::VectorXd& velocity,
                             Eigen::VectorXd& acceleration);
    
    void getTrajectoryAtTimeNonSync(double time,
                                    Eigen::VectorXd& position,
                                    Eigen::VectorXd& velocity,
                                    Eigen::VectorXd& acceleration);
    
    int getDofNum() const { return dofNum_; }

    void setPlannerSyncMode(bool isSync)
    {
        isSync_ = isSync;
        if (isSync) {
            inputPtr_->synchronization = ruckig::Synchronization::Time;
        } else {
            inputPtr_->synchronization = ruckig::Synchronization::None;
        }
    }

private:
    // 常规成员 
    size_t dofNum_{3};                     // 自由度数量

    Eigen::VectorXd current_pose_;          // 当前位姿
    Eigen::VectorXd current_velocity_;      // 当前速度
    Eigen::VectorXd current_acceleration_;  // 当前加速度

    Eigen::VectorXd target_pose_;           // 目标位姿
    
    // ruckig 相关成员
    std::unique_ptr<ruckig::InputParameter<0>> inputPtr_;
    std::unique_ptr<ruckig::Ruckig<0>> ruckigPlannerPtr_;
    std::unique_ptr<ruckig::Trajectory<0>> trajectoryPtr_;
    
    bool isSync_{false};
    bool isVecMode_{false};

    // 用于非同步的预设时间功能，相关的成员变量
    std::vector<ruckig::InputParameter<2>> inputVec_;
    std::vector<ruckig::Ruckig<2>> ruckigPlannerVec_;
    std::vector<ruckig::Trajectory<2>> trajectoryVec_;

    // 保存最大最小值
    Eigen::VectorXd minVelocity_, maxVelocity_;
    Eigen::VectorXd minAcceleration_, maxAcceleration_;
    Eigen::VectorXd minJerk_, maxJerk_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
