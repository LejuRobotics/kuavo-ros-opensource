
#pragma once

#include <cstddef>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "ruckig/ruckig.hpp"

namespace ocs2 {
namespace mobile_manipulator {

// 支持自定义自由度的ruckig规划器封装类

class KinemicLimitFilter {
public:
    KinemicLimitFilter(int dofNum, double dt);

    ~KinemicLimitFilter() = default;

    // 更新数值
    Eigen::VectorXd update(const Eigen::VectorXd& data);

    // 获取一阶/二阶导数
    const Eigen::VectorXd& getFirstOrderDerivative() const;
    const Eigen::VectorXd& getSecondOrderDerivative() const;

    // 设置值约束
    void setValueLimit(const Eigen::VectorXd& minLimit, const Eigen::VectorXd& maxLimit);

    // 设置运动学约束
    void setFirstOrderDerivativeLimit(const Eigen::VectorXd& limit);
    void setFirstOrderDerivativeLimit(const Eigen::VectorXd& minLimit, const Eigen::VectorXd& maxLimit);
    void setSecondOrderDerivativeLimit(const Eigen::VectorXd& limit);
    void setSecondOrderDerivativeLimit(const Eigen::VectorXd& minLimit, const Eigen::VectorXd& maxLimit);
    void setThirdOrderDerivativeLimit(const Eigen::VectorXd& limit);

    // 重置滤波器
    void reset(const Eigen::VectorXd& initialValue);

    /**
     * @brief Reset a contiguous subset of DOFs without disturbing the others.
     *
     * The selected positions are synchronized to @p values and their velocity
     * and acceleration histories are cleared.  The corresponding internal
     * Ruckig trajectories are invalidated, so the next update starts from the
     * supplied state.  State belonging to DOFs outside the selected range is
     * left unchanged.
     *
     * This class is not internally synchronized.  Callers must use the same
     * external lock that protects update() when resetRange() can run from a
     * different thread.
     *
     * @param offset First DOF to reset.
     * @param values New positions for [offset, offset + values.size()).
     * @return true on success; false for an empty, non-finite, or out-of-range
     *         request.  A rejected request does not change any state.
     */
    bool resetRange(std::size_t offset, const Eigen::VectorXd& values);

private:
    // 常规成员 
    size_t dofNum_{3};                      // 自由度数量
    Eigen::VectorXd prevData_;              // 上一时刻数据
    Eigen::VectorXd prevDataFirstOrder_;    // 上一时刻数据的一阶导数
    Eigen::VectorXd prevDataSecondOrder_;   // 上一时刻数据的二阶导数

    // ruckig 相关成员
    std::vector<ruckig::InputParameter<1>> inputVec_;
    std::vector<ruckig::OutputParameter<1>> outputVec_;
    std::vector<ruckig::Ruckig<1>> ruckigVec_;

    // 值约束相关
    bool hasValueLimits_;
    Eigen::VectorXd minValues_;
    Eigen::VectorXd maxValues_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
