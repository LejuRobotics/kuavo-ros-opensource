
#include <pinocchio/fwd.hpp>  // 前向声明
#include <pinocchio/multibody/model.hpp>  // 完整定义
#include <ocs2_core/misc/LoadData.h>

#include "humanoid_wheel_interface/filters/jointCmdLimiter.h"

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

    // joint velocity limits
    vector_t qvelLowerBound = vector_t::Zero(armDim);
    vector_t qvelUpperBound = vector_t::Zero(armDim);

    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", qvelLowerBound);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", qvelUpperBound);

    std::cerr << "[jointCmdLimiter] qvel lowerBound: " << qvelLowerBound.transpose() << '\n';
    std::cerr << "[jointCmdLimiter] qvel upperBound: " << qvelUpperBound.transpose() << '\n';

    jointPosFilter_ptr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(armDim, dt_);
    jointVelFilter_ptr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(armDim, dt_);

    jointPosFilter_ptr_->setValueLimit(qposLowerBound, qposUpperBound);
    jointPosFilter_ptr_->setFirstOrderDerivativeLimit(qvelUpperBound);
    jointVelFilter_ptr_->setValueLimit(qvelLowerBound, qvelUpperBound);
}

// 指令限制
void jointCmdLimiter::update(Eigen::VectorXd& qposCmd, Eigen::VectorXd& qvelCmd)
{
    // 第一次更新初始化
    static bool firstRun = true;
    if(firstRun)
    {
        jointPosFilter_ptr_->reset(qposCmd);
        jointVelFilter_ptr_->reset(qvelCmd);

        firstRun = false;
    }

    qposCmd = jointPosFilter_ptr_->update(qposCmd);
    qvelCmd = jointVelFilter_ptr_->update(qvelCmd);
}

}  // namespace mobile_manipulator
}  // namespace ocs2