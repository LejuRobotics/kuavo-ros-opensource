#pragma once

#include <memory>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>

#include "humanoid_wheel_interface/ManipulatorModelInfo.h"
#include "humanoid_wheel_interface/reference_manager/MobileManipulatorReferenceManager.h"

namespace ocs2 {

namespace mobile_manipulator {
/**
 * 肩部收紧成本: 把配置的肩关节 (如 zarm_l1/l2 与 zarm_r1/r2) 以自适应权重拉向参考轨迹中的肩部姿态。
 *
 *   cost = sum_i weight_i * alpha(arm_i) * (q_i - q_ref_i)^2
 *
 * 其中 alpha ∈ [0,1] 由参考管理器暴露 (1 = 锁肩/收紧, 0 = 释放), 使得小幅度摆动时
 * 肩部尽量保持不动, 而肘腕接近限位或大幅度运动时权重自动下降、允许肩部参与。
 */
class shoulderTightenCost final : public StateCost {
 public:
  /**
   * @param shoulderStateIndices : 肩关节在 MPC 状态向量中的下标 (左右臂)
   * @param armIndices           : 每个下标属于哪只手臂 (0=左, 1=右)
   * @param shoulderWeights      : 每个肩关节的最大权重 (alpha=1 时的权重)
   * @param referenceManager     : 提供每臂的 alpha 与肩关节参考姿态 (期望状态轨迹)
   */
  shoulderTightenCost(std::vector<size_t> shoulderStateIndices,
                      std::vector<int> armIndices,
                      const vector_t& shoulderWeights,
                      const MobileManipulatorReferenceManager& referenceManager);

  ~shoulderTightenCost() override = default;

  shoulderTightenCost* clone() const override { return new shoulderTightenCost(*this); }

  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  shoulderTightenCost(const shoulderTightenCost& other) = default;

  std::vector<size_t> shoulderStateIndices_;
  std::vector<int> armIndices_;
  vector_t shoulderWeights_;
  const MobileManipulatorReferenceManager& referenceManager_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
