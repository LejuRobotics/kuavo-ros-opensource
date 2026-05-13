
#include "humanoid_wheel_interface/motion_planner/cmdVelPlannerWithRuckig.h"

namespace ocs2 {
namespace mobile_manipulator {

cmdVelPlannerWithRuckig::cmdVelPlannerWithRuckig(int dofNum) 
{
    if(dofNum <= 0)
    {
        ROS_WARN_STREAM("Invalid DOF number: " << dofNum << ", using default 3 DOF");
        dofNum = 3;
    }
    
    dofNum_ = static_cast<size_t>(dofNum);

    // 初始化向量
    current_pose_ = Eigen::VectorXd::Zero(dofNum_);
    current_velocity_ = Eigen::VectorXd::Zero(dofNum_);
    current_acceleration_ = Eigen::VectorXd::Zero(dofNum_);

    // 初始化 ruckig 输入参数
    inputVec_.resize(dofNum_);
    ruckigPlannerVec_.resize(dofNum_);
    trajectoryVec_.resize(dofNum_);
    
    // 设置默认约束
    Eigen::VectorXd default_acc_limits = Eigen::VectorXd::Constant(dofNum_, 0.6);
    Eigen::VectorXd default_jerk_limits = Eigen::VectorXd::Constant(dofNum_, 0.3);
    
    setAccelerationLimits(default_acc_limits, -default_acc_limits);
    setJerkLimits(default_jerk_limits);
    
    for(size_t i = 0; i < dofNum_; ++i)
    {
        inputVec_[i].enabled = {true};  // 启用该自由度
        inputVec_[i].control_interface = ruckig::ControlInterface::Velocity;
    }
}

void cmdVelPlannerWithRuckig::setTargetVelocity(const Eigen::VectorXd& velocity) {
    // 自由度检查
    if (velocity.size() != dofNum_) {
        ROS_ERROR_STREAM("Target velocity dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << velocity.size());
        return;
    }
    target_velocity_ = velocity;
    target_velocity_ = target_velocity_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdVelPlannerWithRuckig::setCurrentPose(const Eigen::VectorXd& pose) {
    // 自由度检查
    if (pose.size() != dofNum_) {
        ROS_ERROR_STREAM("Current pose dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << pose.size());
        return;
    }
    current_pose_ = pose;
    current_pose_ = current_pose_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdVelPlannerWithRuckig::setCurrentVelocity(const Eigen::VectorXd& velocity) {
    // 自由度检查
    if (velocity.size() != dofNum_) {
        ROS_ERROR_STREAM("Current velocity dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << velocity.size());
        return;
    }
    current_velocity_ = velocity;
    current_velocity_ = current_velocity_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdVelPlannerWithRuckig::setCurrentAcceleration(const Eigen::VectorXd& acceleration) {
    // 自由度检查
    if (acceleration.size() != dofNum_) {
        ROS_ERROR_STREAM("Current acceleration dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << acceleration.size());
        return;
    }
    current_acceleration_ = acceleration;
    current_acceleration_ = current_acceleration_.unaryExpr([](double x) {    // 只保留3位小数，避免数值误差过大导致的差分计算问题
        return std::round(x * 1000.0) / 1000.0;
    });
}

void cmdVelPlannerWithRuckig::setAccelerationLimits(const Eigen::VectorXd& max_acceleration, 
                                                    const Eigen::VectorXd& max_deceleration) {
    // 自由度检查
    if (max_acceleration.size() != dofNum_ || max_deceleration.size() != dofNum_) {
        ROS_ERROR_STREAM("Acceleration limits dimension mismatch! Expected: " << dofNum_ 
                         << ", Max acceleration: " << max_acceleration.size() 
                         << ", Max deceleration: " << max_deceleration.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        inputVec_[i].max_acceleration = {max_acceleration[i]};
        inputVec_[i].min_acceleration = {max_deceleration[i]};  // 注意：这里使用max_deceleration作为min_acceleration
    }
}

void cmdVelPlannerWithRuckig::setJerkLimits(const Eigen::VectorXd& max_jerk) {
    // 自由度检查
    if (max_jerk.size() != dofNum_) {
        ROS_ERROR_STREAM("Jerk limits dimension mismatch! Expected: " << dofNum_ 
                         << ", Got: " << max_jerk.size());
        return;
    }
    
    for (size_t i = 0; i < dofNum_; ++i) {
        inputVec_[i].max_jerk = {max_jerk[i]};
    }
}

void cmdVelPlannerWithRuckig::calcTrajectory(void) 
{
    bool allSuccess = true;
    // 为每个自由度计算轨迹
    for (size_t i = 0; i < dofNum_; ++i)
    {
        // 设置当前状态和目标状态
        inputVec_[i].current_position = {current_pose_[i]};
        inputVec_[i].current_velocity = {current_velocity_[i]};
        inputVec_[i].current_acceleration = {current_acceleration_[i]};
        
        inputVec_[i].target_velocity = {target_velocity_[i]};  // 默认目标速度为0
        inputVec_[i].target_acceleration = {0.0};  // 默认目标加速度为0

        // 计算轨迹
        ruckig::Result result = ruckigPlannerVec_[i].calculate(inputVec_[i], trajectoryVec_[i]);

        if (result != ruckig::Result::Finished && result != ruckig::Result::Working) {
            ROS_ERROR_STREAM("Ruckig trajectory calculation failed for DOF " << i 
                            << " with error code: " << static_cast<int>(result));
            allSuccess = false;
            continue;
        }
    }

    if (!allSuccess) {
        ROS_ERROR("Some DOF trajectory calculations failed!");
    }
}

void cmdVelPlannerWithRuckig::getTrajectoryAtTime(double time,
                                                   Eigen::VectorXd& position,
                                                   Eigen::VectorXd& velocity,
                                                   Eigen::VectorXd& acceleration) 
{
    // 检查是否已计算轨迹
    if (trajectoryVec_.empty() || trajectoryVec_[0].get_duration() < 0) {
        ROS_ERROR("Trajectory not calculated yet! Call calcTrajectory() first.");
        return;
    }

    // 检查输出向量维度
    if (position.size() != dofNum_) position.resize(dofNum_);
    if (velocity.size() != dofNum_) velocity.resize(dofNum_);
    if (acceleration.size() != dofNum_) acceleration.resize(dofNum_);

    // 获取每个自由度在指定时间点的状态
    for (size_t i = 0; i < dofNum_; ++i)
    {
        std::array<double, 1> pos, vel, acc;

        // 对于已经完成轨迹的自由度，仍然可获取后续轨迹
        double queryTime = time;

        trajectoryVec_[i].at_time(queryTime, pos, vel, acc);

        position[i] = pos[0];
        velocity[i] = vel[0];
        acceleration[i] = acc[0];
    }
}

}  // namespace mobile_manipulator
}  // namespace ocs2