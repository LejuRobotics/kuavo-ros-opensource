/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_wheel_interface/common/Types.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.h"
#include "humanoid_wheel_interface/ManipulatorModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

enum class IKType {
  NULL_IK = 0,    // 不计算IK，直接返回初始值
  LEFT_HAND_ONLY_IK = 1,
  RIGHT_HAND_ONLY_IK = 2,
  LEFT_WHOLE_BODY_IK = 3,
  RIGHT_WHOLE_BODY_IK = 4
};

class InverseKinematics
{
public:
  void setParam(std::shared_ptr<PinocchioInterface> pinocchioInterface, std::shared_ptr<ManipulatorModelInfo> info, int legDim = 4)
  {
    pinocchio_interface_ = pinocchioInterface;
    info_ = info;
    qr_.setThreshold(0.01);

    baseDim_ = info_->stateDim - info_->armDim;
    legDim_ = legDim;
    singleArmDim_ = (info_->armDim - legDim_) / 2;
  }

  vector_t computeHandOnlyIK(vector_t init_q, int hand, vector3_t des_hand_linear_xyz, vector3_t des_hand_eular_zyx, bool isMix = false);
  vector_t computeWholeBodyIK(vector_t init_q, int hand, vector3_t des_hand_linear_xyz, vector3_t des_hand_eular_zyx, bool isMix = false);

  /************************************ 参数修改内置函数 ***************************************************/
  void setTotalTimeDesired(double totalTimeDesired) { if(totalTimeDesired >= 1.0) {totalTimeDesired_ = totalTimeDesired;} }
  void setMaxAttempts(int maxAttempts) { if(maxAttempts <= 10) {maxAttempts_ = maxAttempts;} }
  void setLinearErrorMax(scalar_t linearErrorMax) { if(linearErrorMax >= 0.0002) {linearErrorMax_ = linearErrorMax;} }
  void setAngularErrorMax(scalar_t angularErrorMax) { if(angularErrorMax >= 0.0003) {angularErrorMax_ = angularErrorMax;} }
  scalar_t getLiearErrorMax() const { return linearErrorMax_; }
  scalar_t getAngularErrorMax() const { return angularErrorMax_; }
  scalar_t getBestLinearError() const { return bestLinearError_; }
  scalar_t getBestAngularError() const { return bestAngularError_; }
  bool isBestSolutionWithinThreshold() const { return (bestLinearError_ <= linearErrorMax_) && 
                                                      (bestAngularError_ <= angularErrorMax_); }
  bool isBestSolutionWithinPosThreshold() const { return (bestLinearError_ <= linearErrorMax_); }
  /******************************************************************************************************/

private:

  /************************************ ik 内置解算函数 ***************************************************/
  vector_t computeTranslationIK(vector_t init_q, vector3_t des_hand_linear_xyz, size_t frameId);
  vector_t computeRotationIK(vector_t init_q, matrix3_t des_hand_R_des, size_t frameId);
  vector_t computeRotationIK(vector_t init_q, vector3_t des_hand_eular_zyx, size_t frameId);
  vector_t computeSimpleIK(vector_t init_q, vector3_t des_hand_linear_xyz, 
                           vector3_t des_hand_eular_zyx, size_t frameId);
  void changeJointActiveIndices(IKType ikType);
  /******************************************************************************************************/

  std::shared_ptr<PinocchioInterface> pinocchio_interface_;
  std::shared_ptr<ManipulatorModelInfo> info_;

  Eigen::ColPivHouseholderQR<matrix_t> qr_;

  // 逆解的基础变量信息
  int baseDim_ = 3;  // base的DOF数，默认为 3（x,y,yaw），不参与IK计算
  int legDim_ = 4;   // 每条腿的DOF数，默认为 4，whole-body IK时参与计算，hand-only IK时不参与计算
  int singleArmDim_ = 7; // 单臂的DOF数，默认为 7

  // 计算IK时可迭代部分的标志位
  IKType ikType_ = IKType::NULL_IK;
  std::vector<int> ikJointIndices_;  // 参与IK计算的关节索引列表，根据ikType_设置

  // 逆解伴随的额外信息
  double totalTimeDesired_ = 1.0;  // 期望的总计算时间（秒）
  int maxAttempts_ = 5;  // 最大尝试次数
  scalar_t linearErrorMax_ = 0.001;  // 线性误差阈值（米）
  scalar_t angularErrorMax_ = 0.05;  // 角度误差阈值（弧度）
  
  scalar_t bestLinearError_ = std::numeric_limits<scalar_t>::max();
  scalar_t bestAngularError_ = std::numeric_limits<scalar_t>::max();

};

}  // namespace mobile_manipulator
}  // namespace ocs2
