#include "humanoid_wheel_interface/motion_planner/posePlannerTimedScheduler.h"
#include <cmath>

namespace ocs2 {
namespace mobile_manipulator {

// 获取时间同步的ruckig位置规划器列表
const std::vector<std::shared_ptr<cmdPosePlannerWithRuckig>>& 
posePlannerTimedScheduler::getTimedPlannerPosePtrVec() const
{
    return timedPlannerPosePtrVec_;
}

// 添加时间同步的ruckig位置规划器
void posePlannerTimedScheduler::addTimedPlannerPosePtr(
    const std::shared_ptr<cmdPosePlannerWithRuckig>& plannerPtr)
{
    timedPlannerPosePtrVec_.push_back(plannerPtr);
    stateDiffVec_.resize(timedPlannerPosePtrVec_.size());
}

// 设置时间同步器中的状态信息
void posePlannerTimedScheduler::setTimedPlannerStates(
    const std::vector<Eigen::VectorXd>& currentPose)
{
    size_t numPlanners = timedPlannerPosePtrVec_.size();
    if (currentPose.size() != numPlanners) {
        throw std::runtime_error("Size mismatch when setting timed planner states.");
    }

    static bool isFirstRun{true};
    if(isFirstRun)
    {
        currentPos_.resize(currentPose.size());
        currentVel_.resize(currentPose.size());
        currentAcc_.resize(currentPose.size());
        isFirstRun = false;
    }

    static double currentTime = 0.0;

    currentPos_ = currentPose;

    for (size_t i = 0; i < numPlanners; ++i) {
        int dofNum = timedPlannerPosePtrVec_[i]->getDofNum();
        currentPos_[i] = currentPos_[i].unaryExpr([](double x) {    // 只保留4位小数，避免数值误差过大导致的差分计算问题
            return std::round(x * 10000.0) / 10000.0;
        });
        /******************************** 计算差分 ************************************/
        stateDiffVec_[i].differentiate(currentPos_[i], currentTime, currentVel_[i], currentAcc_[i]);
        /*****************************************************************************/
        currentAcc_[i] = Eigen::VectorXd::Zero(dofNum);

        timedPlannerPosePtrVec_[i]->setCurrentPose(currentPose[i]);
        timedPlannerPosePtrVec_[i]->setCurrentVelocity(currentVel_[i]);
        timedPlannerPosePtrVec_[i]->setCurrentAcceleration(Eigen::VectorXd::Zero(dofNum)); // 这里暂时不使用加速度信息，直接设置为零
    }
    currentTime += diffDt_;
}

void posePlannerTimedScheduler::setTimedPlanner_dStates(
    const std::vector<Eigen::VectorXd>& currentVelocity)
{
    size_t numPlanners = timedPlannerPosePtrVec_.size();
    if (currentVelocity.size() != numPlanners) {
        throw std::runtime_error("Size mismatch when setting timed planner states.");
    }

    for (size_t i = 0; i < numPlanners; ++i) {
        timedPlannerPosePtrVec_[i]->setCurrentVelocity(currentVelocity[i]);
    }
}

void posePlannerTimedScheduler::setTimedPlanner_ddStates(
    const std::vector<Eigen::VectorXd>& currentAcceleration)
{
    size_t numPlanners = timedPlannerPosePtrVec_.size();
    if (currentAcceleration.size() != numPlanners) {
        throw std::runtime_error("Size mismatch when setting timed planner states.");
    }

    for (size_t i = 0; i < numPlanners; ++i) {
        timedPlannerPosePtrVec_[i]->setCurrentAcceleration(currentAcceleration[i]);
    }
}

void posePlannerTimedScheduler::getTimedPlannerStates(int8_t plannerIndex, 
                                                      Eigen::VectorXd& currentPose, 
                                                      Eigen::VectorXd& currentVelocity, 
                                                      Eigen::VectorXd& currentAcceleration)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        ROS_ERROR_STREAM("Invalid planner index in getTimedPlannerStates.");
        return;
    }

    currentPose = currentPos_[plannerIndex];
    currentVelocity = currentVel_[plannerIndex];
    currentAcceleration = currentAcc_[plannerIndex];
}

// 计算时间同步的轨迹，返回预计运动时间
double posePlannerTimedScheduler::calcTimedTrajectory(int8_t plannerIndex, 
                                                     Eigen::VectorXd cmdVec, 
                                                     double desiredTime)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        ROS_ERROR_STREAM("Invalid planner index in calcTimedTrajectory.");
        return -1.0;
    }

    // 判断 cmdVec 维度是否匹配
    if (cmdVec.size() != getTimedPlannerDofNum(plannerIndex)) {
        ROS_ERROR_STREAM("Command vector dimension mismatch in calcTimedTrajectory.");
        return -1.0;
    }

    timedPlannerPosePtrVec_[plannerIndex]->setTargetPose(cmdVec);
    return timedPlannerPosePtrVec_[plannerIndex]->calcTrajectory(desiredTime);
}

// 更新指定索引的速度限制
void posePlannerTimedScheduler::updateTimedPlannerVelocityLimits(
    int8_t plannerIndex,
    const Eigen::VectorXd& max_velocity,
    const Eigen::VectorXd& min_velocity)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        throw std::runtime_error("Invalid planner index in updateTimedPlannerVelocityLimits.");
    }
    timedPlannerPosePtrVec_[plannerIndex]->setVelocityLimits(max_velocity, min_velocity);
}

// 更新指定索引的加速度限制
void posePlannerTimedScheduler::updateTimedPlannerAccelerationLimits(
    int8_t plannerIndex,
    const Eigen::VectorXd& max_acceleration,
    const Eigen::VectorXd& max_deceleration)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        throw std::runtime_error("Invalid planner index in updateTimedPlannerAccelerationLimits.");
    }
    timedPlannerPosePtrVec_[plannerIndex]->setAccelerationLimits(max_acceleration, max_deceleration);
}

// 更新指定索引的加加速度限制
void posePlannerTimedScheduler::updateTimedPlannerJerkLimits(
    int8_t plannerIndex,
    const Eigen::VectorXd& max_jerk)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        throw std::runtime_error("Invalid planner index in updateTimedPlannerJerkLimits.");
    }
    timedPlannerPosePtrVec_[plannerIndex]->setJerkLimits(max_jerk);
}

// 设置指定规划器的同步模式
void posePlannerTimedScheduler::setTimedPlannerSyncMode(int8_t plannerIndex, bool isSync)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        throw std::runtime_error("Invalid planner index in setTimedPlannerSyncMode.");
    }
    timedPlannerPosePtrVec_[plannerIndex]->setPlannerSyncMode(isSync);
}

// 获取指定规划器的自由度数量
int posePlannerTimedScheduler::getTimedPlannerDofNum(int8_t plannerIndex) const
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        throw std::runtime_error("Invalid planner index in getTimedPlannerDofNum.");
    }
    return timedPlannerPosePtrVec_[plannerIndex]->getDofNum();
}

// 获取指定规划器在指定时间的轨迹信息
void posePlannerTimedScheduler::getTimedPlannerTrajectoryAtTime(
    int8_t plannerIndex,
    double time,
    Eigen::VectorXd& position,
    Eigen::VectorXd& velocity,
    Eigen::VectorXd& acceleration)
{
    if (plannerIndex < 0 || static_cast<size_t>(plannerIndex) >= timedPlannerPosePtrVec_.size()) {
        throw std::runtime_error("Invalid planner index in getTimedPlannerTrajectoryAtTime.");
    }
    timedPlannerPosePtrVec_[plannerIndex]->getTrajectoryAtTime(time, position, velocity, acceleration);
}

}  // namespace mobile_manipulator
}  // namespace ocs2