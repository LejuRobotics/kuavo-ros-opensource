
#include <pinocchio/fwd.hpp>  // 前向声明
#include <pinocchio/multibody/model.hpp>  // 完整定义
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ocs2_core/misc/LoadData.h>

#include "humanoid_wheel_interface/filters/jointCmdLimiter.h"

namespace ocs2 {
namespace mobile_manipulator {

jointCmdLimiter::jointCmdLimiter(int dofNum, PinocchioInterface pinocchioInterface, 
                                std::string taskFile, const ManipulatorModelInfo& info, 
                                double dt, int armNum):
pinocchioInterface_(pinocchioInterface), info_(info)
{
    dt_ = dt;
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    baseDim_ = info.stateDim - info.armDim;
    const auto& model = pinocchioInterface.getModel();

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

    // joint torque limits
    const int lowerJointNum = info.armDim - armNum; 
    vector_t torqueLimit(lowerJointNum + armNum/2);
    loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimit);
    torqueLimit_.setZero(info_.armDim);
    torqueLimit_ << torqueLimit.head(lowerJointNum), 
                    torqueLimit.tail(armNum/2), 
                    torqueLimit.tail(armNum/2);

    std::cerr << "[jointCmdLimiter] torque Limit: " << torqueLimit_.transpose() << std::endl;

    jointPosFilter_ptr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(armDim, dt_);
    jointVelFilter_ptr_ = std::make_shared<mobile_manipulator::KinemicLimitFilter>(armDim, dt_);

    jointPosFilter_ptr_->setValueLimit(qposLowerBound, qposUpperBound);
    jointPosFilter_ptr_->setFirstOrderDerivativeLimit(qvelUpperBound);
    jointVelFilter_ptr_->setValueLimit(qvelLowerBound, qvelUpperBound);

    // joint acc limit
    const std::string prefix = "safeAccLimit.";
    bool safeAccEnable = false;
    loadData::loadPtreeValue(pt, safeAccEnable, prefix + "enable", true);
    if(safeAccEnable)
    {
        loadData::loadPtreeValue(pt, safetyFactor_, prefix + "factor", true);

        vector_t safeAccBound(lowerJointNum + armNum/2);
        loadData::loadEigenMatrix(taskFile, prefix + "bound", safeAccBound);
        vector_t safeAccBoundFull(info.armDim);
        safeAccBoundFull << safeAccBound.head(lowerJointNum), 
                            safeAccBound.tail(armNum/2),
                            safeAccBound.tail(armNum/2);
        // 安全系数
        if (safetyFactor_ > 0.0 && safetyFactor_ <= 1.0)
        {
            safeAccBoundFull.array() *= safetyFactor_;
        }
        else
        {
            throw std::runtime_error("[jointCmdLimiter] safetyFactor_ must be in (0, 1]");
        }
        // 设置到约束中
        jointPosFilter_ptr_->setSecondOrderDerivativeLimit(safeAccBoundFull);
        jointVelFilter_ptr_->setFirstOrderDerivativeLimit(safeAccBoundFull);
    }

    // enable realTime Safe Acc
    loadData::loadPtreeValue(pt, realTimeAccEnable_, prefix + "realTimeAccEnable", true);
}

// 仅约束位置指令在限制范围内（硬裁剪）
void jointCmdLimiter::clipPositionCommand(Eigen::VectorXd& qposCmd)
{
    const auto& model = pinocchioInterface_.getModel();
    const int armDim = info_.armDim;
    
    // 获取位置上下限
    const vector_t qposLowerBound = model.lowerPositionLimit.tail(armDim);
    const vector_t qposUpperBound = model.upperPositionLimit.tail(armDim);
    
    // 逐元素裁剪到边界
    for (int i = 0; i < armDim; ++i) {
        if (qposCmd[i] < qposLowerBound[i]) {
            qposCmd[i] = qposLowerBound[i];
        } else if (qposCmd[i] > qposUpperBound[i]) {
            qposCmd[i] = qposUpperBound[i];
        }
    }
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
        qvelObs_.setZero(info_.armDim);
        qposObs_ = qposCmd;

        firstRun = false;
    }

    if(realTimeAccEnable_)
    {
        updateMaxAcc(qposCmd);
    }

    qposCmd = jointPosFilter_ptr_->update(qposCmd);
    qvelCmd = jointVelFilter_ptr_->update(qvelCmd);

    qvelObs_ = (qposCmd - qposObs_) / dt_;
    qposObs_ = qposCmd;
}

// 实时更新最大加速度(使用效果差)
void jointCmdLimiter::updateMaxAcc(const Eigen::VectorXd& qposCmd)
{
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    
    vector_t qposObsFull = vector_t::Zero(info_.stateDim);
    qposObsFull << Eigen::Vector3d::Zero(), qposObs_;
    vector_t qvelObsFull = vector_t::Zero(info_.stateDim);
    qvelObsFull << Eigen::Vector3d::Zero(), qvelObs_;

    pinocchio::crba(model, data, qposObsFull);
    pinocchio::nonLinearEffects(model, data, qposObsFull, qvelObsFull);

    vector_t qAccUpperBound(info_.armDim);
    vector_t qAccLowerBound(info_.armDim);

    for (int i = 0; i < info_.armDim; ++i)
    {
        int armIdx = baseDim_ + i;
        double M_ii = data.M(armIdx, armIdx);  // 直接从质量矩阵取对角元
        double b_i = data.nle[armIdx];    // 非线性项
        double M_ii_inv = 1.0 / M_ii;
        // 正向加速度上限
        double tau_available_pos = torqueLimit_[i] - b_i;
        qAccUpperBound[i] = tau_available_pos * M_ii_inv;
        // 负向加速度下限
        double tau_available_neg = -torqueLimit_[i] - b_i;
        qAccLowerBound[i] = tau_available_neg * M_ii_inv;
    }
    // 安全系数
    if (safetyFactor_ > 0.0 && safetyFactor_ <= 1.0)
    {
        qAccUpperBound.array() *= safetyFactor_;
        qAccLowerBound.array() *= safetyFactor_;
    }
    for (int i = 0; i < info_.armDim; ++i)
    {
        int armIdx = baseDim_ + i;
        double M_ii = data.M(armIdx, armIdx);  // 直接从质量矩阵取对角元
        // std::cout << "M(" << i << "," << i << "):" << M_ii << std::endl;
    }
    // std::cout << "qAccUpperBound: " << qAccUpperBound.transpose() << std::endl;
    // std::cout << "qAccLowerBound: " << qAccLowerBound.transpose() << std::endl;
    
    // 更新限制器
    jointPosFilter_ptr_->setSecondOrderDerivativeLimit(qAccLowerBound, qAccUpperBound);
    jointVelFilter_ptr_->setFirstOrderDerivativeLimit(qAccLowerBound, qAccUpperBound);
}

}  // namespace mobile_manipulator
}  // namespace ocs2