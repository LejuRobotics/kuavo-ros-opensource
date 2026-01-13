#pragma once

#include <iostream>
#include <vector>
#include <cstdint>
#include <Eigen/Geometry>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/com_position_constraint.h"
#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"
#include "drake/common/autodiff.h"
// #include "utils.h"

namespace HighlyDynamic
{
  // ==================== 求解模式枚举（便于阅读/配置） ====================
  // 说明：
  // - 外部仍然可以直接按 bitmask 给 IKParams::constraint_mode 赋值（保持兼容）。
  // - 该枚举只是给“常用模式”一个更清晰的名字，底层数值仍对应 constraint_mode 的 bit 含义：
  //     bit1(0x02): 位置 hard(1)/soft(0)
  //     bit0(0x01): 姿态 hard(1)/soft(0)
  //     bit2(0x04): 三点位置模式(1)/传统位姿模式(0)
  enum class ConstraintMode : uint8_t
  {
    // 传统位姿模式（bit2=0）
    PosSoft_OriSoft = 0x00,  // 00：位软 + 姿软
    PosSoft_OriHard = 0x01,  // 01：位软 + 姿硬
    PosHard_OriSoft = 0x02,  // 10：位硬 + 姿软
    PosHard_OriHard = 0x03,  // 11：位硬 + 姿硬

    // 三点位置模式（bit2=1）：不加任何姿态项；用 3 个点的位置项间接约束姿态
    ThreePoint_Soft  = 0x04, // 04：三点全软（P0/P1/P2 全 cost）
    ThreePoint_Mixed = 0x06, // 06：三点混合（P0 hard，P1/P2 cost）
  };

  // ==================== 外部->内部 constraint_mode 映射 ====================
  // 需求：外部用户仍按 bitmask（二进制）设置 constraint_mode，但内部把“软软(00)”与“软硬(01)”对调。
  // 目的：让外部默认值 0（常见于消息未赋值/配置缺省）时，内部实际走“位软+姿硬”，减少误用风险。
  //
  // 规则（仅对传统位姿模式的低两位生效）：
  // - 00 <-> 01 互换
  // - 10/11 不变
  // - bit2（三点模式）及更高位保持不变
  inline uint8_t MapExternalConstraintModeToInternal(uint8_t external_mode)
  {
    const uint8_t low2 = external_mode & 0x03;                 // bit1/bit0
    const uint8_t rest = external_mode & static_cast<uint8_t>(~uint8_t{0x03}); // bit2 及更高位

    if (low2 == 0x00)
      return static_cast<uint8_t>(rest | 0x01);
    if (low2 == 0x01)
      return static_cast<uint8_t>(rest | 0x00);
    return external_mode;
  }

  enum class HandSide
  {
    LEFT,
    RIGHT,
    BOTH
  };
  HandSide intToHandSide(int value) {
    if (value < 0 || value > 2) 
      throw std::invalid_argument("Invalid value for HandSide");
    return static_cast<HandSide>(value);
  }
  typedef std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> FramePoseVec;
  struct IKParams
  {
    // snopt params
    double major_optimality_tol{1e-3};
    double major_feasibility_tol{1e-3};
    double minor_feasibility_tol{1e-3};
    double major_iterations_limit{100};
    // constraint and cost params
    double oritation_constraint_tol{1e-3};
    double pos_constraint_tol{1e-3}; // work when pos_cost_weight > 0.0
    double pos_cost_weight{100}; // NOT work if pos_cost_weight <= 0.0
    // 使用 IKParams::constraint_mode 的 bit 来选择模式（以当前 plantIK.cc 实现为准）：
    //   bit1 (0x02): 位置 hard(1)/soft(0)
    //   bit0 (0x01): 姿态 hard(1)/soft(0)
    //   bit2 (0x04): 三点位置模式(1)/传统位姿模式(0)
    //
    // 传统位姿模式（bit2=0，只看 bit1/bit0）：
    //   00->0: 位软 + 姿软   (AddPositionCost + AddOrientationCost)
    //   01->1: 位软 + 姿硬   (AddPositionCost + AddOrientationConstraint)
    //   10->2: 位硬 + 姿软   (AddPositionConstraint + AddOrientationCost)
    //   11->3: 位硬 + 姿硬   (AddPositionConstraint + AddOrientationConstraint)
    //
    // 三点位置模式（bit2=1，不添加任何姿态约束/代价；仅对每只手的末端帧添加 3 个点的位置项）：
    //   - 三个点（在 eef frame 内）：
    //       P0 = eef 原点 (0,0,0)
    //       P1 = link7 原点在 eef 中的平移（由固定变换计算）
    //       P2 = virtual thumb 点 (0.15, 0, 0)  [URDF: zarm_*7_virtual_thumb_joint_ee]
    //   04->4: 三点“全软” (P0/P1/P2 全是 AddPositionCost)
    //   06->6: 三点“混合” (P0 用 AddPositionConstraint，P1/P2 用 AddPositionCost)
    uint8_t constraint_mode{1};
  };
  class CoMIK
  {
  public:
    CoMIK(){};
    CoMIK(drake::multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name);

    bool solve(const FramePoseVec &pose, const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol, IKParams params = IKParams());

    std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q, HandSide side);

  private:
    drake::solvers::Binding<drake::solvers::Constraint> AddCoMPositionConstraint(drake::multibody::InverseKinematics &ik, const Eigen::Vector3d &r_des);

    drake::multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<drake::systems::Context<double>> plant_context_;
    std::vector<std::string> frames_name_;
    Eigen::VectorXd prev_q_sol;
    int nq_;
  };

} // namespace drake
