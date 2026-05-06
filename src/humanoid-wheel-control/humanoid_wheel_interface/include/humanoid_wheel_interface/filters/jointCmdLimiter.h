
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "humanoid_wheel_interface/filters/KinemicLimitFilter.h"

namespace ocs2 {
namespace mobile_manipulator {

// 自定义关节限制器
class jointCmdLimiter {
public:
    jointCmdLimiter(int dofNum, PinocchioInterface pinocchioInterface, 
                    std::string taskFile, const ManipulatorModelInfo& info, 
                    double dt, int armNum);

    ~jointCmdLimiter() = default;

    // 硬约束位置
    void clipPositionCommand(Eigen::VectorXd& qposCmd);

    // 更新数值
    void update(Eigen::VectorXd& qposCmd, Eigen::VectorXd& qvelCmd);

    // 实时更新加速度
    void updateMaxAcc(const Eigen::VectorXd& qposCmd);

private:
    // 常规成员
    double dt_ = 0.001;
    int baseDim_ = 3;

    // 动力学库
    PinocchioInterface pinocchioInterface_;
    const ManipulatorModelInfo& info_;

    // 扭矩限制
    vector_t torqueLimit_;

    // 安全系数
    double safetyFactor_ = 0.1;

    // 实时计算加速度边界
    bool realTimeAccEnable_{false};

    // 存储上次输出
    vector_t qposObs_;
    vector_t qvelObs_;

    // 插值类
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  jointPosFilter_ptr_;
    std::shared_ptr<mobile_manipulator::KinemicLimitFilter>  jointVelFilter_ptr_;

};

}  // namespace mobile_manipulator
}  // namespace ocs2
