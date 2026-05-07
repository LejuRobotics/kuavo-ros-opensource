
#include "humanoid_wheel_interface/filters/jointCmdLimiter.h"
#include <pinocchio/fwd.hpp>  // 前向声明
#include <pinocchio/multibody/model.hpp>  // 完整定义
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace mobile_manipulator {

jointCmdLimiter::jointCmdLimiter(int dofNum, PinocchioInterface pinocchioInterface, 
                                std::string taskFile, const ManipulatorModelInfo& info, 
                                double dt)
{
    dt_ = dt;
    auto& model = pinocchioInterface.getModel();

    const int armDim = info.armDim;
    const vector_t qposLowerBound = model.lowerPositionLimit.tail(armDim);
    const vector_t qposUpperBound = model.upperPositionLimit.tail(armDim);

    std::cerr << "[jointCmdLimiter] qpos lowerBound: " << qposLowerBound.transpose() << '\n';
    std::cerr << "[jointCmdLimiter] qpos upperBound: " << qposUpperBound.transpose() << '\n';

    qposMin_ = qposLowerBound;
    qposMax_ = qposUpperBound;

    // joint velocity limits
    vector_t qvelLowerBound = vector_t::Zero(armDim);
    vector_t qvelUpperBound = vector_t::Zero(armDim);

    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", qvelLowerBound);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", qvelUpperBound);

    std::cerr << "[jointCmdLimiter] qvel lowerBound: " << qvelLowerBound.transpose() << '\n';
    std::cerr << "[jointCmdLimiter] qvel upperBound: " << qvelUpperBound.transpose() << '\n';

    qvelMin_ = qvelLowerBound;
    qvelMax_ = qvelUpperBound;

}

// 指令限制
void jointCmdLimiter::update(Eigen::VectorXd& qposCmd, Eigen::VectorXd& qvelCmd)
{
    if (qposCmd.size() != qvelCmd.size() && 
        qposCmd.size() != qposMin_.size() && 
        qvelCmd.size() != qvelMin_.size()) {
        throw std::runtime_error("[jointCmdLimiter] qposCmd and qvelCmd dimension mismatch");
    }

    // 第一次更新初始化
    static bool firstRun = true;
    if(firstRun)
    {
        lastQpos_ = qposCmd;
        lastQvel_ = qvelCmd;

        firstRun = false;
    }

    // 速度限制直接采用预设最大最小值
    qvelCmd = qvelCmd.cwiseMax(qvelMin_).cwiseMin(qvelMax_);

    // 位置限制采用增量限制和值限制两种
    Eigen::VectorXd deltaPos = qposCmd - lastQpos_;

    Eigen::VectorXd maxVelDelta = qvelMax_ * dt_;
    Eigen::VectorXd minVelDelta = qvelMin_ * dt_;

    deltaPos = deltaPos.cwiseMax(minVelDelta).cwiseMin(maxVelDelta);

    Eigen::VectorXd limitedPos = lastQpos_ + deltaPos;

    qposCmd = limitedPos.cwiseMax(qposMin_).cwiseMin(qposMax_);

    // 保存状态
    lastQpos_ = qposCmd;
    lastQvel_ = qvelCmd;
}

}  // namespace mobile_manipulator
}  // namespace ocs2